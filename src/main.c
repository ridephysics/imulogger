/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <misc/printk.h>
#include <misc/byteorder.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <disk_access.h>
#include <fs.h>
#include <ff.h>

#define SD_MOUNT_POINT "/SD:"
static FATFS fat_fs;
static struct fs_mount_t mp = {
    .type = FS_FATFS,
    .fs_data = &fat_fs,
    .mnt_point = SD_MOUNT_POINT,
};

static struct bt_uuid_128 uuid_service = BT_UUID_INIT_128(
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 uuid_chr_active = BT_UUID_INIT_128(
    0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static uint8_t logger_active = 0;

static ssize_t active_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
            void *buf, u16_t len, u16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &logger_active, sizeof(logger_active));
}

static ssize_t active_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
             const void *buf, u16_t len, u16_t offset,
             u8_t flags)
{
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len != 1) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    uint8_t val = *((uint8_t*)buf);
    if (val > 1) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    logger_active = val;

    return 1;
}

static struct bt_gatt_attr vnd_attrs[] = {
    BT_GATT_PRIMARY_SERVICE(&uuid_service),

    BT_GATT_CHARACTERISTIC(&uuid_chr_active.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        active_read, active_write, NULL
    ),
    BT_GATT_CUD("active", BT_GATT_PERM_READ),
};

static struct bt_gatt_service vnd_svc = BT_GATT_SERVICE(vnd_attrs);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};

static void connected(struct bt_conn *conn, u8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
    } else {
        printk("Connected\n");
    }
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
    printk("Disconnected (reason %u)\n", reason);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    bt_gatt_service_register(&vnd_svc);

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }

    printk("Advertising successfully started\n");
}

static int init_fs(void)
{
    int err;
    static const char *disk_pdrv = "SD";
    u64_t memory_size_mb;
    u32_t block_count;
    u32_t block_size;

    err = disk_access_init(disk_pdrv);
    if (err) {
        printk("Storage init ERROR (err %d)!\n", err);
        return -1;
    }

    err = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &block_count);
    if (err) {
        printk("Unable to get sector count (err %d)\n", err);
        return -1;
    }
    printk("Block count %u\n", block_count);

    err = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &block_size);
    if (err) {
        printk("Unable to get sector size (err %d)\n", err);
        return -1;
    }
    printk("Sector size %u\n", block_size);

    memory_size_mb = (u64_t)block_count * block_size;
    printk("Memory Size(MB) %u\n", (u32_t)(memory_size_mb>>20));

    err = fs_mount(&mp);
    if (err != FR_OK) {
        printk("Error mounting disk. (err %d)\n", err);
        return -1;
    }

    printk("Disk mounted.\n");
    return 0;
}

void main(void)
{
    int err;

    err = init_fs();
    if (err) {
        printk("FS init failed (err %d)\n", err);
        return;
    }

    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    bt_conn_cb_register(&conn_callbacks);

    while (1) {
        k_sleep(MSEC_PER_SEC);
    }
}
