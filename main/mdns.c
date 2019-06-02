#include "main.h"
#include <mdns.h>

#define CROSSLOG_TAG "main"
#include <crosslog.h>

/* these strings match tcpip_adapter_if_t enumeration */
static const char * if_str[] = {"STA", "AP", "ETH", "MAX"};

/* these strings match mdns_ip_protocol_t enumeration */
static const char * ip_protocol_str[] = {"V4", "V6", "MAX"};

static void mdns_print_results(mdns_result_t *results) {
    mdns_result_t *r = results;
    mdns_ip_addr_t *a = NULL;
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
        while (a) {
            if (a->addr.type == IPADDR_TYPE_V6) {
                printf("  AAAA: " IPV6STR "\n", IPV62STR(a->addr.u_addr.ip6));
            }
            else {
                printf("  A   : " IPSTR "\n", IP2STR(&(a->addr.u_addr.ip4)));
            }
            a = a->next;
        }
        r = r->next;
    }
}

static int mdns_process_results(struct mqtt_ctx *ctx, mdns_result_t *results) {
    mdns_result_t *r;
    mdns_ip_addr_t *a;

    for (r = results; r; r = r->next){
        for (a = r->addr; a; a = a->next) {
            if (a->addr.type == IPADDR_TYPE_V4) {
                ctx->addr = a->addr;
                ctx->port = r->port;
                return 0;
            }
        }
    }

    return -1;
}

static int query_mdns_service(struct mqtt_ctx *ctx, const char *service_name, const char *proto)
{
    int rc;

    CROSSLOGD("Query PTR: %s.%s.local", service_name, proto);

    mdns_result_t *results = NULL;
    esp_err_t err = mdns_query_ptr(service_name, proto, 3000, 20,  &results);
    if (err) {
        CROSSLOGE("Query Failed: %s", esp_err_to_name(err));
        return -1;
    }
    if (!results) {
        CROSSLOGW("No results found!");
        return -1;
    }

    mdns_print_results(results);
    rc = mdns_process_results(ctx, results);
    mdns_query_results_free(results);

    if (rc) {
        CROSSLOGW("none of the results matched");
    }
    else {
        CROSSLOGD("mdns query successful");
    }

    return rc;
}

static void mdns_task(void *_ctx) {
    int rc;
    struct mqtt_ctx *ctx = _ctx;

    for (;;) {
        rc = query_mdns_service(ctx, "_imulogger_mqtt_broker", "_tcp");
        if (rc == 0) {
            rc = uev_event_post(&ctx->w_resolved);
            if (rc) {
                CROSSLOG_ERRNO("uev_event_post");
                CROSSLOG_ASSERT(0);
            }

            break;
        }

        vTaskDelay((5 * 1000) / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

static void resolved_cb(uev_t *w, void *_ctx, int events) {
    struct mqtt_ctx *ctx = _ctx;
    CROSSLOGD("%s", __func__);
    mdns_resolved_cb(ctx);
}

int init_mdns(uev_ctx_t *uev, struct mqtt_ctx *ctx) {
    int rc;
    BaseType_t xrc;

    memset(ctx, 0, sizeof(*ctx));
    ctx->uev = uev;

    // mqtt
    rc = uev_event_init(uev, &ctx->w_resolved, resolved_cb, ctx);
    if (rc) {
        CROSSLOG_ERRNO("uev_event_init");
        CROSSLOG_ASSERT(0);
    }

    ESP_ERROR_CHECK( mdns_init() );
    ESP_ERROR_CHECK( mdns_hostname_set("imulogger") );
    ESP_ERROR_CHECK( mdns_instance_name_set("imulogger") );
    ESP_ERROR_CHECK( mdns_service_add("FTP", "_ftp", "_tcp", 21, NULL, 0) );

    xrc = xTaskCreate(mdns_task, "mdns_query", 4096, ctx, ESP_TASK_MAIN_PRIO, NULL);
    if (xrc != pdPASS) {
        uev_event_stop(&ctx->w_resolved);
        return -1;
    }

    return 0;
}
