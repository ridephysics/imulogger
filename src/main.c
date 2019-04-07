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
#include <atomic.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <disk_access.h>
#include <fs.h>
#include <ff.h>

#include <crossi2c/zephyr.h>
#include <em7180.h>

#define SD_MOUNT_POINT "/SD:"
#define SD_MOUNT_POINT_LEN sizeof(SD_MOUNT_POINT) - 1
static FATFS fat_fs;
static struct fs_mount_t mp = {
    .type = FS_FATFS,
    .fs_data = &fat_fs,
    .mnt_point = SD_MOUNT_POINT,
};

#define I2C_DEV DT_I2C_0_NAME
static struct crossi2c_bus i2cbus;
static struct em7180 em7180;

static struct bt_uuid_128 uuid_service = BT_UUID_INIT_128(
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 uuid_chr_active = BT_UUID_INIT_128(
    0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 uuid_chr_logname = BT_UUID_INIT_128(
    0xf2, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 uuid_chr_fs_path = BT_UUID_INIT_128(
    0x01, 0xdf, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 uuid_chr_fs_data = BT_UUID_INIT_128(
    0x02, 0xdf, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_uuid_128 uuid_chr_fs_unlink = BT_UUID_INIT_128(
    0x03, 0xdf, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

#define LOGNAME_PREFIX_LEN SD_MOUNT_POINT_LEN + 1
static atomic_t logger_active = ATOMIC_INIT(0);
static struct k_sem logger_sem;
static char logname[LOGNAME_PREFIX_LEN + 20 + 1];

static char fs_path[SD_MOUNT_POINT_LEN + 20 + 1];
static uint8_t fs_dir_valid;
static struct fs_dir_t fs_dir;
static uint8_t fs_file_valid;
static struct fs_file_t fs_file;

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

    if (val) {
        if (atomic_get(&logger_active)) {
            printk("logger is already enabled\b");
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
        }

        k_sem_give(&logger_sem);
        atomic_set(&logger_active, 1);
    }
    else {
        if (!atomic_get(&logger_active)) {
            printk("logger is already disabled\b");
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
        }

        atomic_set(&logger_active, 0);
    }

    return len;
}

static ssize_t logname_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
            void *buf, u16_t len, u16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, logname + LOGNAME_PREFIX_LEN, strlen(logname + LOGNAME_PREFIX_LEN));
}

static ssize_t logname_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
             const void *buf, u16_t len, u16_t offset,
             u8_t flags)
{
    if (offset + len > sizeof(logname) - LOGNAME_PREFIX_LEN - 1) {
        strcpy(logname + LOGNAME_PREFIX_LEN, "default");
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    memcpy(logname + LOGNAME_PREFIX_LEN + offset, buf, len);
    logname[LOGNAME_PREFIX_LEN + offset + len] = '\0';

    printk("new logname: %s\n", logname);

    return len;
}

static ssize_t fs_path_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
             const void *buf, u16_t len, u16_t offset,
             u8_t flags)
{
    if (offset + len > sizeof(fs_path) - SD_MOUNT_POINT_LEN - 1) {
        fs_path[SD_MOUNT_POINT_LEN] = '\0';
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    memcpy(fs_path + SD_MOUNT_POINT_LEN + offset, buf, len);
    fs_path[SD_MOUNT_POINT_LEN + offset + len] = '\0';

    printk("new fs path: %s\n", fs_path);

    if (fs_dir_valid) {
        fs_closedir(&fs_dir);
        fs_dir_valid = 0;
    }

    if (fs_file_valid) {
        fs_close(&fs_file);
        fs_file_valid = 0;
    }

    return len;
}

static ssize_t fs_data_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
            void *buf, u16_t len, u16_t offset)
{
    int err;

    if (offset != 0) {
        printk("non-zero offset '%u' is not allowed\n", offset);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (logger_active) {
        printk("reading is not allowed while logging\n");
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    if (!fs_dir_valid && !fs_file_valid) {
        // list files in root dir
        if (fs_path[SD_MOUNT_POINT_LEN] == '/' && fs_path[SD_MOUNT_POINT_LEN + 1] == '\0') {
            printk("open root dir\n");

            err = fs_opendir(&fs_dir, fs_path);
            if (err) {
                printk("can't open root dir: %d\n", err);
                return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
            }

            fs_dir_valid = 1;
        }

        // read file contents
        else if (fs_path[SD_MOUNT_POINT_LEN]) {
            static struct fs_dirent entry;
            printk("open file '%s'\n", fs_path);

            err = fs_stat(fs_path, &entry);
            if (err) {
                printk("can't stat file '%s': %d\n", fs_path, err);
                return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
            }

            err = fs_open(&fs_file, fs_path);
            if (err) {
                printk("can't open file: %d\n", err);
                return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
            }

            fs_file_valid = 1;
        }

        else {
            printk("no path set\n");
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
        }

        // reset path so we don't read it twice
        fs_path[SD_MOUNT_POINT_LEN] = '\0';
    }

    if (fs_dir_valid && fs_file_valid) {
        printk("BUG: we have both a dir and a file\n");

        fs_closedir(&fs_dir);
        fs_dir_valid = 0;

        fs_close(&fs_file);
        fs_file_valid = 0;

        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    if (fs_dir_valid) {
        static struct fs_dirent entry;
        size_t namelen;

        for (;;) {
            err = fs_readdir(&fs_dir, &entry);
            if (err) {
                printk("readdir failed: %d\n", err);

                fs_closedir(&fs_dir);
                fs_dir_valid = 0;

                return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
            }
            namelen = strlen(entry.name);

            if (entry.type != FS_DIR_ENTRY_FILE) {
                printk("entry '%s' is not a file, skip\n", entry.name);
                continue;
            }

            if (namelen > 20) {
                printk("name '%s' is too long, skip\n", entry.name);
                continue;
            }

            if (namelen == 0) {
                printk("end of dir\n");

                fs_closedir(&fs_dir);
                fs_dir_valid = 0;

                return 0;
            }

            break;
        }

        return bt_gatt_attr_read(conn, attr, buf, len, offset, entry.name, namelen);
    }

    else if(fs_file_valid) {
        err = fs_read(&fs_file, buf, len);
        if (err < 0) {
            printk("fs_read error: %d\n", err);

            fs_close(&fs_file);
            fs_file_valid = 0;

            return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
        }

        if (err == 0) {
            printk("end of file\n");

            fs_close(&fs_file);
            fs_file_valid = 0;

            return 0;
        }

        return err;
    }

    else {
        printk("BUG: we have no open file or dir\n");
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
}

static ssize_t fs_unlink_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
             const void *buf, u16_t len, u16_t offset,
             u8_t flags)
{
    int err;

    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len != 1) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    if (*((uint8_t*)buf) != 0x01) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    if (logger_active) {
        printk("deleting is not allowed while logging\n");
        return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    printk("delete path: %s\n", fs_path);
    err = fs_unlink(fs_path);
    fs_path[SD_MOUNT_POINT_LEN] = '\0';

    if (err) {
        printk("unlink failed: %d\n", err);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    return len;
}

static struct bt_gatt_attr vnd_attrs[] = {
    BT_GATT_PRIMARY_SERVICE(&uuid_service),

    BT_GATT_CHARACTERISTIC(&uuid_chr_active.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        active_read, active_write, NULL
    ),
    BT_GATT_CUD("active", BT_GATT_PERM_READ),

    BT_GATT_CHARACTERISTIC(&uuid_chr_logname.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        logname_read, logname_write, NULL
    ),
    BT_GATT_CUD("logname", BT_GATT_PERM_READ),

    BT_GATT_CHARACTERISTIC(&uuid_chr_fs_path.uuid,
        BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE,
        NULL, fs_path_write, NULL
    ),
    BT_GATT_CUD("fs_path", BT_GATT_PERM_READ),

    BT_GATT_CHARACTERISTIC(&uuid_chr_fs_data.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        fs_data_read, NULL, NULL
    ),
    BT_GATT_CUD("fs_data", BT_GATT_PERM_READ),

    BT_GATT_CHARACTERISTIC(&uuid_chr_fs_unlink.uuid,
        BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE,
        NULL, fs_unlink_write, NULL
    ),
    BT_GATT_CUD("fs_unlink", BT_GATT_PERM_READ),
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

    strcpy(fs_path, SD_MOUNT_POINT);
    strcpy(logname, SD_MOUNT_POINT "/default");

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

static int init_imu(void) {
    int err;
    struct device *i2c_dev;

    i2c_dev = device_get_binding(I2C_DEV);
    if (!i2c_dev) {
        printk("I2C: Device driver not found.\n");
        return -1;
    }

    err = crossi2c_zephyr_create(&i2cbus, i2c_dev);
    if (err) {
        printk("crossi2c init failed (err %d)\n", err);
        return -1;
    }

    err = em7180_create(&em7180, &i2cbus);
    if (err)  {
        printk("em7180 creation failed (err %d)\n", err);
        return -1;
    }

    err = em7180_init(&em7180);
    if (err)  {
        printk("em7180 init failed (err %d)\n", err);
        return -1;
    }

    em7180_set_algorithm(&em7180, EM7180_AS_STANDBY);

    return 0;
}

static void do_log(void) {
    int err;
    static struct fs_file_t log_file;
    uint8_t alg_status;
    uint8_t event_status;
    uint8_t error_reg;
    s64_t uptime;
    u32_t counter;
    static uint8_t data_raw[50];

    printk("start logging\n");

    err = fs_open(&log_file, logname);
    if (err) {
        printk("can't open '%s': %d\n", logname, err);
        return;
    }

    em7180_set_algorithm(&em7180, 0x00);

    for (;;) {
        if (!atomic_get(&logger_active)) {
            printk("logger got disabled\n");
            break;
        }

        err = em7180_get_algorithm_status(&em7180, &alg_status);
        if (err) {
            printk("Unable to get algorithm status (err %d)\n", err);
            continue;
        }

        err = em7180_get_event_status(&em7180, &event_status);
        if (err) {
            printk("Unable to get event status (err %d)\n", err);
            continue;
        }

        if (event_status & EM7180_EVENT_ERROR) {
            err = em7180_get_error_register(&em7180, &error_reg);
            if (err) {
                printk("Unable to get error register (err %d)\n", err);
                continue;
            }
            em7180_print_error(error_reg);
            continue;
        }

        if (!event_status) {
            continue;
        }

        uptime = k_uptime_get();
        counter = k_cycle_get_32();

        err = em7180_get_data_all_raw(&em7180, data_raw);
        if (err) {
            printk("Unable to get raw data (err %d)\n", err);
            continue;
        }

        err = fs_write(&log_file, &uptime, sizeof(uptime));
        if (err < 0 || (size_t)err != sizeof(uptime)) {
            printk("can't write uptime\n");
            break;
        }

        err = fs_write(&log_file, &counter, sizeof(counter));
        if (err < 0 || (size_t)err != sizeof(counter)) {
            printk("can't write counter\n");
            break;
        }

        err = fs_write(&log_file, &alg_status, sizeof(alg_status));
        if (err < 0 || (size_t)err != sizeof(alg_status)) {
            printk("can't write alg_status\n");
            break;
        }

        err = fs_write(&log_file, &event_status, sizeof(event_status));
        if (err < 0 || (size_t)err != sizeof(event_status)) {
            printk("can't write event_status\n");
            break;
        }

        err = fs_write(&log_file, data_raw, sizeof(data_raw));
        if (err < 0 || (size_t)err != sizeof(data_raw)) {
            printk("can't write raw data\n");
            break;
        }
    }

    em7180_set_algorithm(&em7180, EM7180_AS_STANDBY);

    err = fs_close(&log_file);
    if (err) {
        printk("can't close logfile: %d\n", err);
    }

    printk("logging finished\n");
}

void main(void)
{
    int err;

    k_sem_init(&logger_sem, 0, 1);

    err = init_fs();
    if (err) {
        printk("FS init failed (err %d)\n", err);
        return;
    }

    err = init_imu();
    if (err) {
        printk("IMU init failed (err %d)\n", err);
        return;
    }

    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    bt_conn_cb_register(&conn_callbacks);

    for (;;) {
        if (k_sem_take(&logger_sem, K_FOREVER)) {
            printk("sem error\n");
            continue;
        }

        do_log();
    }
}
