#include <uev/uev.h>
#include <tcpip_adapter.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <driver/uart.h>
#include <esp_vfs_dev.h>
#include <esp32/rom/spi_flash.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <mdns.h>

#define CROSSLOG_TAG "main"
#include <crosslog.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

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

/* these strings match tcpip_adapter_if_t enumeration */
static const char * if_str[] = {"STA", "AP", "ETH", "MAX"};

/* these strings match mdns_ip_protocol_t enumeration */
static const char * ip_protocol_str[] = {"V4", "V6", "MAX"};

static void mdns_print_results(mdns_result_t * results){
    mdns_result_t * r = results;
    mdns_ip_addr_t * a = NULL;
    int i = 1;
    int t;

    while (r){
        printf("%d: Interface: %s, Type: %s\n", i++, if_str[r->tcpip_if], ip_protocol_str[r->ip_protocol]);
        if (r->instance_name) {
            printf("  PTR : %s\n", r->instance_name);
        }
        if (r->hostname) {
            printf("  SRV : %s.local:%u\n", r->hostname, r->port);
        }
        if (r->txt_count) {
            printf("  TXT : [%u] ", r->txt_count);
            for(t=0; t<r->txt_count; t++){
                printf("%s=%s; ", r->txt[t].key, r->txt[t].value?r->txt[t].value:"NULL");
            }
            printf("\n");
        }

        a = r->addr;
        while (a){
            if (a->addr.type == IPADDR_TYPE_V6) {
                printf("  AAAA: " IPV6STR "\n", IPV62STR(a->addr.u_addr.ip6));
            }
            else {
                printf("  A   : " IPSTR "\n", IP2STR(&(a->addr.u_addr.ip4)));

                // TODO: start mqtt client
            }
            a = a->next;
        }
        r = r->next;
    }

}

static void query_mdns_service(const char * service_name, const char * proto)
{
    CROSSLOGI("Query PTR: %s.%s.local", service_name, proto);

    mdns_result_t * results = NULL;
    esp_err_t err = mdns_query_ptr(service_name, proto, 3000, 20,  &results);
    if(err){
        CROSSLOGE("Query Failed: %s", esp_err_to_name(err));
        return;
    }
    if(!results){
        CROSSLOGW("No results found!");
        return;
    }

    mdns_print_results(results);
    mdns_query_results_free(results);
}

static void mdns_task(void *ctx) {
    for (;;) {
        query_mdns_service("_imulogger_mqtt_broker", "_tcp");
        vTaskDelay((5 * 1000) / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void init_mdns(void) {
    static StaticTask_t taskbuf;
    static StackType_t stackbuf[4096];
    TaskHandle_t task;

    ESP_ERROR_CHECK( mdns_init() );
    ESP_ERROR_CHECK( mdns_hostname_set("imulogger") );
    ESP_ERROR_CHECK( mdns_instance_name_set("imulogger") );
    ESP_ERROR_CHECK( mdns_service_add("FTP", "_ftp", "_tcp", 21, NULL, 0) );

    task = xTaskCreateStatic(mdns_task, "mdns_query", ARRAY_SIZE(stackbuf), NULL,
        ESP_TASK_MAIN_PRIO, stackbuf, &taskbuf);
    CROSSLOG_ASSERT(task);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
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
        init_mdns();
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

    // wifi
    init_softap();

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
