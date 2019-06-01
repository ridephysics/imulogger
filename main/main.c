#include <uev/uev.h>
#include <tcpip_adapter.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <driver/uart.h>
#include <esp_vfs_dev.h>

#define CROSSLOG_TAG "main"
#include <crosslog.h>

static uev_ctx_t _uev;
static uev_ctx_t * const uev = &_uev;

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

    rc = uev_init(uev);
    if (rc) {
        CROSSLOGE("can't init uev");
        CROSSLOG_ASSERT(0);
    }

    rc = uev_run(uev, 0);
    if (rc) {
        CROSSLOGE("uev_run returned %d", rc);
    }

    uev_exit(uev);

    CROSSLOGD("main task finished");
}
