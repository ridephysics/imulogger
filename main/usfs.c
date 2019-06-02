#include "main.h"
#include <crossi2c/esp32.h>
#include <em7180.h>

#define CROSSLOG_TAG "main"
#include <crosslog.h>

static struct crossi2c_bus i2cbus;
static struct em7180 em7180;

void init_usfs(void) {
    int rc;
    esp_err_t erc;
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

    rc = em7180_init(&em7180);
    CROSSLOG_ASSERT(rc == 0);

    CROSSLOGI("USFS initialized");
}
