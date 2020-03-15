#include "main.h"
#include <stdatomic.h>
#include <crossi2c/esp32.h>
#include <em7180.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/param.h>
#include <usfs.h>

#ifdef CONFIG_IMULOGGER_SENTRAL_PASSTHROUGH
#include <usfs/bmp280.h>
#include <inv_mpu.h>
#endif

#define CROSSLOG_TAG "main"
#include <crosslog.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define BROADCAST_MAGIC_PTR ((void*)0xdeadbeef)

static volatile TaskHandle_t task = NULL;
static StaticTask_t taskbuf;
static StackType_t stackbuf[4096];

static volatile TaskHandle_t task_write = NULL;
static StaticTask_t taskbuf_write;
static StackType_t stackbuf_write[4096];

static StaticSemaphore_t lockbuf;
static SemaphoreHandle_t lock = NULL;
static char imu_filename[256];
static atomic_bool logging_enabled;
static atomic_char imu_status;
static atomic_uint samplerate;
static atomic_bool do_broadcast;
static struct crossi2c_bus i2cbus;
static struct em7180 em7180;
#ifdef CONFIG_IMULOGGER_SENTRAL_PASSTHROUGH
static struct bmp280_dev bmp280;
static struct mpu_state_s mpu9250;
#endif

#define SAMPLE_QUEUE_LEN 512
static uint8_t sample_queue_storage[USFS_TOTAL_SAMPLESZ * SAMPLE_QUEUE_LEN];
static StaticQueue_t sample_queue_buf;
static QueueHandle_t sample_queue = NULL;

static FILE* volatile current_file = NULL;
static StaticSemaphore_t file_lockbuf;
static SemaphoreHandle_t file_lock = NULL;

static struct list_node listeners = LIST_INITIAL_VALUE(listeners);
static uev_t w_enabled;
static uev_t w_status;
static uev_t w_samplerate;

static int sockfd;
static struct sockaddr_in sa;

static int usfs_write(FILE *f, const void *data, size_t len) {
    int rc;

    if (f == BROADCAST_MAGIC_PTR) {
        rc = sendto(sockfd, data, len, 0, (struct sockaddr *) &sa, sizeof(sa));
        if (rc < 0) {
            if (errno != ENOMEM) {
                CROSSLOG_ERRNO("sendto");
            }
            return -1;
        }

        if (rc != len) {
            CROSSLOGE("short write");
            return -1;
        }
    }
    else if(f) {
        if (fwrite(data, len, 1, f) != 1) {
            CROSSLOGE("fwrite failed");
            return -1;
        }
    }

    return 0;
}

static int usfs_init(void) {
    int rc;

#ifdef CONFIG_IMULOGGER_SENTRAL_PASSTHROUGH
    int8_t brc;
    struct bmp280_config conf;

    rc = usfs_bmp280_create(&bmp280, &i2cbus);
    if (rc) {
        CROSSLOGE("can't create bmp280");
        return -1;
    }

    rc = mpu_create(&mpu9250, MPU_TYPE_MPU6500, MAG_TYPE_AK8963, &i2cbus);
    if (rc) {
        CROSSLOGE("can't create mpu9250");
        return -1;
    }

    rc = em7180_reset_request(&em7180);
    if (rc) {
        CROSSLOGE("can't reset em7180");
        return -1;
    }

    rc = em7180_set_run_mode(&em7180, false);
    if (rc) {
        CROSSLOGE("can't disable em7180 run mode");
        return -1;
    }

    rc = em7180_passthrough_enter(&em7180);
    if (rc) {
        CROSSLOGE("can't enter em7180 passthrough mode");
        return -1;
    }

    brc = bmp280_init(&bmp280);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_init: %d", brc);
        return -1;
    }

    brc = bmp280_get_config(&conf, &bmp280);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_get_config: %d", brc);
        return -1;
    }

    conf.filter = BMP280_FILTER_OFF;
    conf.os_temp = BMP280_OS_1X;
    conf.os_pres = BMP280_OS_4X;
    conf.odr = BMP280_ODR_0_5_MS;

    brc = bmp280_set_config(&conf, &bmp280);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_set_config: %d", brc);
        return -1;
    }

    brc = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp280);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_set_power_mode: %d", brc);
        return -1;
    }

    rc = mpu_init(&mpu9250);
    if (rc) {
        CROSSLOGE("can't init mpu9250");
        return -1;
    }

    rc = mpu_set_sensors(&mpu9250, INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    if (rc) {
        CROSSLOGE("mpu_set_sensors failed");
        return -1;
    }

    rc = mpu_set_sample_rate(&mpu9250, 200);
    if (rc) {
        CROSSLOGE("mpu_set_sample_rate failed");
        return -1;
    }

    rc = mpu_set_accel_fsr(&mpu9250, 8);
    if (rc) {
        CROSSLOGE("mpu_set_accel_fsr failed");
        return -1;
    }

    rc = mpu_set_gyro_fsr(&mpu9250, 2000);
    if (rc) {
        CROSSLOGE("mpu_set_gyro_fsr failed");
        return -1;
    }
#else
    rc = em7180_init(&em7180);
    if (rc) {
        CROSSLOGE("em7180_init failed");
        return -1;
    }
#endif

    return 0;
}

static int usfs_write_hdr(FILE *f) {
#ifdef CONFIG_IMULOGGER_SENTRAL_PASSTHROUGH
    struct {
        struct mpu_cfg_dump cfg;
        struct bmp280_calib_param calib_param;
    } __attribute__((packed)) data;

    mpu_get_cfg(&mpu9250, &data.cfg);
    memcpy(&data.calib_param, &bmp280.calib_param, sizeof(data.calib_param));

    if (usfs_write(f, &data, sizeof(data))) {
        CROSSLOGE("can't write header");
        return -1;
    }

#else
    // nothing to do
#endif

    return 0;
}

static inline __attribute__((always_inline)) int usfs_getwrite_sample(FILE *f, uint64_t *ptime) {
    int rc;

#ifdef CONFIG_IMULOGGER_SENTRAL_PASSTHROUGH
    int8_t brc;
    uint8_t data[USFS_TOTAL_SAMPLESZ];
    BaseType_t xrc;

    rc = mpu_get_all_data(&mpu9250, &data[0], (uint64_t*)&data[MPU_RAWSZ]);
    if (rc) {
        CROSSLOGE("mpu_get_all_data failed");
        return -1;
    }

    brc = bmp280_get_raw_data(&data[MPU_SAMPLESZ], &bmp280);
    if (brc != BMP280_OK) {
        CROSSLOGE("bmp280_get_raw_data failed");
        return -1;
    }

    *ptime = usfs_get_us();
    *((uint64_t*)&data[MPU_SAMPLESZ + BMP_RAWSZ]) = *ptime;

    if (uxQueueSpacesAvailable(sample_queue) < 1) {
        xQueueReset(sample_queue);
    }

    xrc = xQueueSend(sample_queue, data, portMAX_DELAY);
    if (xrc != pdTRUE) {
        CROSSLOGE("xQueueSend reported full");
        return -1;
    }
#else
    uint8_t sensor_status;
    uint8_t error_reg;

    struct {
        uint64_t time;
        uint8_t alg_status;
        uint8_t event_status;
        uint8_t data_raw[EM7180_RAWDATA_SZ];
    } __attribute__((packed)) data;

    rc = em7180_get_event_status(&em7180, &data.event_status);
    if (rc) {
        CROSSLOGE("Unable to get event status (err %d)", rc);
        return -1;
    }

    // don't read any data if the error flag is set
    if (data.event_status & EM7180_EVENT_ERROR) {
        rc = em7180_get_error_register(&em7180, &error_reg);
        if (rc) {
            CROSSLOGE("Unable to get error register (err %d)", rc);
            return -1;
        }

        em7180_print_error((enum em7180_error)error_reg);
        return -1;
    }

    // update the current algorithm status
    rc = em7180_get_algorithm_status(&em7180, &data.alg_status);
    if (rc) {
        CROSSLOGE("Unable to get algorithm status (err %d)", rc);
        return -1;
    }
    uint8_t alg_status_prev = atomic_exchange(&imu_status, data.alg_status);
    if (alg_status_prev != data.alg_status) {
        uev_event_post(&w_status);
    }

    // print any sensor communication errors
    rc = em7180_get_sensor_status(&em7180, &sensor_status);
    if (rc) {
        CROSSLOGE("Unable to get sensor status (err %d)", rc);
        return -1;
    }
    if (sensor_status) {
        em7180_print_sensor_status(sensor_status);
    }

    // if there are no events, don't do anything
    if (!data.event_status) {
        return -2;
    }

    // get all sensor data
    rc = em7180_get_data_all_raw(&em7180, data.data_raw);
    if (rc) {
        CROSSLOGE("Unable to get raw data (err %d)", rc);
        return -1;
    }

    data.time = usfs_get_us();
    *ptime = data.time;

    // write data
    if (usfs_write(f, &data, sizeof(data))) {
        return -1;
    }
#endif

    return 0;
}

static void usfs_task_fn(void *unused) {
    int rc;
    esp_err_t erc;
    int opt;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 18,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = 19,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master = {
            .clk_speed = 400000,
        }
    };

    erc = i2c_param_config(0, &conf);
    ESP_ERROR_CHECK(erc);

    erc = i2c_driver_install(0, conf.mode, 0, 0, 0);
    ESP_ERROR_CHECK(erc);

    rc = crossi2c_esp32_create(&i2cbus, 0);
    CROSSLOG_ASSERT(rc == 0);

    rc = em7180_create(&em7180, &i2cbus);
    CROSSLOG_ASSERT(rc == 0);

    rc = usfs_init();
    CROSSLOG_ASSERT(rc == 0);

    bzero(&sa, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = inet_addr("255.255.255.255");
    sa.sin_port = htons(8888);

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        CROSSLOG_ERRNO("socket");
        CROSSLOG_ASSERT(0);
    }

    opt = 1;
    rc = setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
    if (rc) {
        CROSSLOG_ERRNO("setsockopt");
        CROSSLOG_ASSERT(0);
    }

    CROSSLOGI("USFS initialized");

    for (;;) {
        FILE *f = NULL;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!atomic_load(&logging_enabled))
            continue;

        CROSSLOGD("logging started");

        if (!atomic_load(&do_broadcast)) {
            rc = sdcard_ref();
            if (rc) {
                CROSSLOGE("can't ref sdcard");
                goto stop_notify;
            }

            xSemaphoreTake(lock, portMAX_DELAY);
            struct stat sb;

            rc = stat(imu_filename, &sb);
            if (rc == 0) {
                CROSSLOGE("file does already exist");
                xSemaphoreGive(lock);
                goto stop_unmount;
            }

            if (errno != ENOENT) {
                CROSSLOG_ERRNO("stat");
                xSemaphoreGive(lock);
                goto stop_unmount;
            }

            f = fopen(imu_filename, "w");
            xSemaphoreGive(lock);
            if (!f) {
                CROSSLOG_ERRNO("fopen");
                goto stop_unmount;
            }
        }
        else {
            f = BROADCAST_MAGIC_PTR;
        }

        xSemaphoreTake(file_lock, portMAX_DELAY);
        current_file = f;
        xSemaphoreGive(file_lock);

        rc = usfs_write_hdr(f);
        if (rc) {
            CROSSLOGE("can't write hdr");
            goto close_file;
        }

        uintptr_t datacnt = 0;
        uint64_t datatime = usfs_get_us();

        while (atomic_load(&logging_enabled)) {
            uint64_t sampletime;

            rc = usfs_getwrite_sample(f, &sampletime);
            if (rc) continue;

            // calculate samplerate
            datacnt++;
            if (sampletime - datatime >= 10 * 1000 * 1000) {
                atomic_store(&samplerate, datacnt);
                datacnt = 0;
                datatime = sampletime;
                uev_event_post(&w_samplerate);
            }
        }

close_file:
        if (f) {
            xQueueReset(sample_queue);
            xSemaphoreTake(file_lock, portMAX_DELAY);
            current_file = NULL;
            xSemaphoreGive(file_lock);

            if (f != BROADCAST_MAGIC_PTR)
                fclose(f);
        }
stop_unmount:
        if (f && f != BROADCAST_MAGIC_PTR)
            sdcard_unref();
stop_notify:
        atomic_store(&logging_enabled, false);
        uev_event_post(&w_enabled);
        CROSSLOGD("logging stopped");
    }

    task = NULL;
    vTaskDelete(NULL);
}

static void ev_enabled_cb(uev_t *w, void *arg, int events) {
    struct usfs_listener *entry;
    struct usfs_listener *tmp;

    bool enabled = atomic_load(&logging_enabled);

    list_for_every_entry_safe(&listeners, entry, tmp, struct usfs_listener, node) {
        if (entry->enabled)
            entry->enabled(entry->ctx, enabled);
    }
}

static void ev_status_cb(uev_t *w, void *arg, int events) {
    struct usfs_listener *entry;
    struct usfs_listener *tmp;

    uint8_t status = atomic_load(&imu_status);

    list_for_every_entry_safe(&listeners, entry, tmp, struct usfs_listener, node) {
        if (entry->status)
            entry->status(entry->ctx, status);
    }
}

static void ev_samplerate_cb(uev_t *w, void *arg, int events) {
    struct usfs_listener *entry;
    struct usfs_listener *tmp;

    unsigned int rate = atomic_load(&samplerate);

    list_for_every_entry_safe(&listeners, entry, tmp, struct usfs_listener, node) {
        if (entry->samplerate)
            entry->samplerate(entry->ctx, rate);
    }
}

static void usfs_task_write_fn(void *unused) {
    BaseType_t xrc;
    uint8_t buf[USFS_TOTAL_SAMPLESZ];

    for (;;) {
        xrc = xQueueReceive(sample_queue, buf, portMAX_DELAY);
        if (xrc != pdTRUE)
            continue;

        xSemaphoreTake(file_lock, portMAX_DELAY);
        if (current_file) {
            usfs_write(current_file, buf, sizeof(buf));
        }
        xSemaphoreGive(file_lock);
    }

    task_write = NULL;
    vTaskDelete(NULL);
}

void init_usfs(uev_ctx_t *uev) {
    int rc;

    rc = snprintf(imu_filename, sizeof(imu_filename), "/sdcard/default.bin");
    CROSSLOG_ASSERT(rc >= 0 && rc < sizeof(imu_filename));

    lock = xSemaphoreCreateMutexStatic(&lockbuf);
    CROSSLOG_ASSERT(lock);

    file_lock = xSemaphoreCreateMutexStatic(&file_lockbuf);
    CROSSLOG_ASSERT(file_lock);

    sample_queue = xQueueCreateStatic(SAMPLE_QUEUE_LEN, USFS_TOTAL_SAMPLESZ,
        sample_queue_storage, &sample_queue_buf);
    assert(sample_queue);

    atomic_init(&logging_enabled, false);
    atomic_init(&imu_status, 0x00);
    atomic_init(&samplerate, 0);
    atomic_init(&do_broadcast, false);

    rc = uev_event_init(uev, &w_enabled, ev_enabled_cb, NULL);
    CROSSLOG_ASSERT(rc == 0);

    rc = uev_event_init(uev, &w_status, ev_status_cb, NULL);
    CROSSLOG_ASSERT(rc == 0);

    rc = uev_event_init(uev, &w_samplerate, ev_samplerate_cb, NULL);
    CROSSLOG_ASSERT(rc == 0);

    task = xTaskCreateStaticPinnedToCore(usfs_task_write_fn, "usfs_write",
        ARRAY_SIZE(stackbuf_write), NULL, ESP_TASK_TIMER_PRIO,
        stackbuf_write, &taskbuf_write, 1);
    CROSSLOG_ASSERT(task);

    task = xTaskCreateStaticPinnedToCore(usfs_task_fn, "usfs",
        ARRAY_SIZE(stackbuf), NULL, ESP_TASK_TIMER_PRIO,
        stackbuf, &taskbuf, 1);
    CROSSLOG_ASSERT(task);
}

bool usfs_is_enabled(void) {
    return atomic_load(&logging_enabled);
}

uint8_t usfs_status(void) {
    return atomic_load(&imu_status);
}

unsigned int usfs_samplerate(void) {
    return atomic_load(&samplerate);
}

void usfs_set_enabled(bool enabled) {
    atomic_store(&logging_enabled, enabled);
    xTaskNotifyGive(task);
    uev_event_post(&w_enabled);
}

int usfs_set_filename(const char *new_filename, size_t new_filename_len) {
    int ret = -1;

    xSemaphoreTake(lock, portMAX_DELAY);

    if (new_filename_len > sizeof(imu_filename) - 1)
        goto unlock;

    memcpy(imu_filename, new_filename, new_filename_len);
    imu_filename[new_filename_len] = '\0';
    ret = 0;

unlock:
    xSemaphoreGive(lock);

    return ret;
}

int ufsfs_get_filename(char *buf, size_t bufsz, size_t *poutlen) {
    int ret = -1;

    xSemaphoreTake(lock, portMAX_DELAY);

    size_t namelen = strlen(imu_filename);
    if (namelen > bufsz)
        goto unlock;

    memcpy(buf, imu_filename, namelen);

    if (poutlen)
        *poutlen = namelen;

    ret = 0;

unlock:
    xSemaphoreGive(lock);

    return ret;
}

bool usfs_is_broadcast(void) {
    return atomic_load(&do_broadcast);
}

void usfs_set_broadcast(bool broadcast) {
    atomic_store(&do_broadcast, broadcast);
}

void usfs_listener_add(struct usfs_listener *listener) {
    list_add_tail(&listeners, &listener->node);
}

void usfs_listener_del(struct usfs_listener *listener) {
    list_delete(&listener->node);
}
