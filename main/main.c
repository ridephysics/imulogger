#include "main.h"
#include <uev/uev.h>
#include <tcpip_adapter.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <driver/uart.h>
#include <esp_vfs_dev.h>
#include <esp32/rom/spi_flash.h>
#include <esp_system.h>
#include <esp_wifi.h>

#define CROSSLOG_TAG "main"
#include <crosslog.h>

static uev_ctx_t _uev;
static uev_ctx_t * const uev = &_uev;
static struct mqtt_ctx _mqttctx;

static void init_console(void)
{
    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
            256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    int rc;

    switch (event_id) {
    case WIFI_EVENT_AP_STACONNECTED: {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        CROSSLOGI("station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
        break;
    }

    case WIFI_EVENT_AP_STADISCONNECTED: {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        CROSSLOGI("station "MACSTR" leave, AID=%d", MAC2STR(event->mac), event->aid);
        break;
    }

    case WIFI_EVENT_AP_START:
        // mdns
        rc = init_mdns(uev, &_mqttctx);
        if (rc) {
            CROSSLOGE("can't init mdns");
        }
        break;

    default:
        break;
    }
}

static void init_softap(void)
{
    int rc;
    uint8_t mac[6];

    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    rc = snprintf((char*)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "imulogger-%02x%02x", mac[4], mac[5]);
    if (rc < 0 || (size_t)rc >= sizeof(wifi_config.ap.ssid)) {
        CROSSLOGE("can't build SSID: %d", rc);
        CROSSLOG_ASSERT(0);
    }

    rc = snprintf((char*)wifi_config.ap.password, sizeof(wifi_config.ap.password), "imusecret-%08x", g_rom_flashchip.device_id);
    if (rc < 0 || (size_t)rc >= sizeof(wifi_config.ap.password)) {
        CROSSLOGE("can't build password: %d", rc);
        CROSSLOG_ASSERT(0);
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    CROSSLOGV("wifi_init_softap finished. SSID:%s password:%s", wifi_config.ap.ssid, wifi_config.ap.password);
}

void app_main(void)
{
    int rc;

    CROSSLOGI("Hello world!");

    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // misc globals
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    init_console();

    // uev
    rc = uev_init(uev);
    if (rc) {
        CROSSLOG_ERRNO("uev_init");
        CROSSLOG_ASSERT(0);
    }

    // wifi
    init_softap();

    rc = uev_run(uev, 0);
    if (rc) {
        CROSSLOGE("uev_run returned %d", rc);
    }

    uev_exit(uev);

    CROSSLOGD("main task finished");
}
