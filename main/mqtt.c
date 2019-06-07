#include "main.h"
#include <tcpip_adapter.h>

#define CROSSLOG_TAG "main"
#include <crosslog.h>

enum imucmd {
    IMUCMD_FULLREPORT = 0x00,
    IMUCMD_FILENAME = 0x01,
    IMUCMD_ENABLED = 0x02,
    IMUCMD_IMUSTATUS = 0x03,
    IMUCMD_SAMPLERATE = 0x04,
    IMUCMD_SDCARD = 0x05,
};

#define TOPIC_CTRL "/imulogger/ctrl"
#define TOPIC_STATUS "/imulogger/status"

static uint8_t outbuf[256];
static bool sdcard_refed = 0;

static bool strequal(const void *s1, size_t s1len, const char *s2) {
    size_t s2len = strlen(s2);

    if (s1len != s2len) {
        return false;
    }

    return !memcmp(s1, s2, s2len);
}

static inline void dumppub(struct mqtt_response_publish *pub) {
    CROSSLOG_HEXDUMP(pub->application_message, pub->application_message_size);
}

static int send_filename(struct mqtt_ctx *ctx) {
    int rc;
    enum MQTTErrors merr;
    size_t filenamelen;

    outbuf[0] = IMUCMD_FILENAME;

    rc = ufsfs_get_filename((char*)&outbuf[1], sizeof(outbuf) - 1, &filenamelen);
    if (rc) {
        CROSSLOGE("can't get filename");
        return -1;
    }

    merr = mqtt_publish(&ctx->client, TOPIC_STATUS, outbuf, 1 + filenamelen, MQTT_PUBLISH_QOS_0);
    if (merr != MQTT_OK) {
        CROSSLOGE("publishing enabled failed: %s", mqtt_error_str(merr));
        return -1;
    }

    return 0;
}

static int send_enabled(struct mqtt_ctx *ctx, bool enabled) {
    enum MQTTErrors merr;

    outbuf[0] = IMUCMD_ENABLED;
    outbuf[1] = enabled;

    merr = mqtt_publish(&ctx->client, TOPIC_STATUS, outbuf, 2, MQTT_PUBLISH_QOS_0);
    if (merr != MQTT_OK) {
        CROSSLOGE("publishing enabled failed: %s", mqtt_error_str(merr));
        return -1;
    }

    return 0;
}

static int send_imustatus(struct mqtt_ctx *ctx, uint8_t status) {
    enum MQTTErrors merr;

    outbuf[0] = IMUCMD_IMUSTATUS;
    outbuf[1] = status;

    merr = mqtt_publish(&ctx->client, TOPIC_STATUS, outbuf, 2, MQTT_PUBLISH_QOS_0);
    if (merr != MQTT_OK) {
        CROSSLOGE("publishing enabled failed: %s", mqtt_error_str(merr));
        return -1;
    }

    return 0;
}

static int send_samplerate(struct mqtt_ctx *ctx, unsigned int samplerate) {
    enum MQTTErrors merr;

    outbuf[0] = IMUCMD_SAMPLERATE;
    outbuf[1] = samplerate & 0xff;
    outbuf[2] = (samplerate >> 8) & 0xff;
    outbuf[3] = (samplerate >> 16) & 0xff;
    outbuf[4] = (samplerate >> 24) & 0xff;

    merr = mqtt_publish(&ctx->client, TOPIC_STATUS, outbuf, 5, MQTT_PUBLISH_QOS_0);
    if (merr != MQTT_OK) {
        CROSSLOGE("publishing enabled failed: %s", mqtt_error_str(merr));
        return -1;
    }

    return 0;
}

static int send_sdcard(struct mqtt_ctx *ctx) {
    enum MQTTErrors merr;

    outbuf[0] = IMUCMD_SDCARD;
    outbuf[1] = sdcard_refed;

    merr = mqtt_publish(&ctx->client, TOPIC_STATUS, outbuf, 2, MQTT_PUBLISH_QOS_0);
    if (merr != MQTT_OK) {
        CROSSLOGE("publishing sdcard failed: %s", mqtt_error_str(merr));
        return -1;
    }

    return 0;
}

static void send_fullreport(struct mqtt_ctx *ctx) {
    send_filename(ctx);
    send_imustatus(ctx, usfs_status());
    send_enabled(ctx, usfs_is_enabled());
    send_samplerate(ctx, usfs_samplerate());
    send_sdcard(ctx);
}

static int set_filename(struct mqtt_ctx *ctx, const char *filename, size_t filenamesz) {
    int rc;

    rc = usfs_set_filename(filename, filenamesz);
    if (rc) {
        CROSSLOGE("can't set filename");
        return -1;
    }

    return send_filename(ctx);
}

static int set_enabled(struct mqtt_ctx *ctx, uint8_t enable) {
    if (enable & ~0x01) {
        CROSSLOGE("enable must be either 0x00 or 0x01, not 0x%02x", enable);
        return -1;
    }

    usfs_set_enabled(!!enable);

    return 0;
}

static void publish_callback(void **pctx, struct mqtt_response_publish *pub)
{
    struct mqtt_ctx *ctx = *pctx;
    const uint8_t *msg = pub->application_message;
    size_t msgsz = pub->application_message_size;
    int rc;

    if (strequal(pub->topic_name, pub->topic_name_size, TOPIC_CTRL)) {
        if (msgsz < 1 ) {
            CROSSLOGE("ctrl msgs must be at least 1 byte");
            dumppub(pub);
            return;
        }

        uint8_t _cmd = msg[0];
        enum imucmd cmd = (enum imucmd)_cmd;
        switch (cmd) {
        case IMUCMD_FULLREPORT:
            if (msgsz != 1) {
                CROSSLOGE("fullreport request has invalid size: %u", msgsz);
                dumppub(pub);
                return;
            }

            CROSSLOGD("fullreport requested");
            send_fullreport(ctx);
            break;

        case IMUCMD_FILENAME: {
            size_t filenamesz = msgsz - 1;
            const char *filename = (const char*)(filenamesz ? &msg[1] : NULL);
            CROSSLOGD("new filename(%zu): %.*s", filenamesz, (int)filenamesz, filename);
            set_filename(ctx, filename, filenamesz);
            break;
        }

        case IMUCMD_ENABLED: {
            if (msgsz != 2) {
                CROSSLOGE("enabled request has invalid size: %u", msgsz);
                dumppub(pub);
                return;
            }

            uint8_t enable = msg[1];
            CROSSLOGD("enable=0x%02x requested", enable);
            set_enabled(ctx, enable);
            break;
        }

        case IMUCMD_SDCARD: {
            if (msgsz != 2) {
                CROSSLOGE("sdcard request has invalid size: %u", msgsz);
                dumppub(pub);
                return;
            }

            uint8_t doref = msg[1];
            CROSSLOGD("doref=0x%02x requested", doref);
            if (doref && !sdcard_refed) {
                rc = sdcard_ref();
                if (rc) {
                    CROSSLOGE("can't ref sdcard");
                    return;
                }
                sdcard_refed = 1;
                send_sdcard(ctx);
            }
            else if (!doref && sdcard_refed) {
                sdcard_unref();
                sdcard_refed = 0;
                send_sdcard(ctx);
            }
            break;
        }

        case IMUCMD_IMUSTATUS:
        default:
            CROSSLOGE("unsupported ctrl cmd: 0x%02x", _cmd);
            dumppub(pub);
            break;
        }
    }

    else {
        CROSSLOGE("unsupported publish topic: %.*s", (int)pub->topic_name_size, (const char *)pub->topic_name);
        dumppub(pub);
        return;
    }
}

static enum MQTTErrors on_connected(struct mqtt_ctx *ctx, int fd) {
    enum MQTTErrors merr;

    CROSSLOGI("connected");

    /* Reinitialize the client. */
    merr = mqtt_reinit(&ctx->client, fd, 
                ctx->sendbuf, sizeof(ctx->sendbuf),
                ctx->recvbuf, sizeof(ctx->recvbuf)
    );
    if (merr != MQTT_OK) {
        CROSSLOGE("mqtt_reinit: %s", mqtt_error_str(merr));
        close(fd);
        return merr;
    }

    /* Send connection request to the broker. */
    merr = mqtt_connect(&ctx->client, "subscribing_client", NULL, NULL, 0, NULL, NULL, 0, 400);
    if (merr != MQTT_OK) {
        CROSSLOGE("mqtt_connect: %s", mqtt_error_str(merr));
        close(fd);
        return merr;
    }

    /* Subscribe to the topic. */
    merr = mqtt_subscribe(&ctx->client, TOPIC_CTRL, 0);
    if (merr != MQTT_OK) {
        CROSSLOGE("mqtt_subscribe: %s", mqtt_error_str(merr));
        close(fd);
        return merr;
    }

    usfs_listener_add(&ctx->listener);
    ctx->listener_registered = true;

    return MQTT_OK;
}

static void sockfd_cb(uev_t *w, void *_ctx, int events) {
    struct mqtt_ctx *ctx = _ctx;
    int rc;
    int err;
    socklen_t sl = sizeof(err);
    int fd = ctx->connect_fd;
    enum MQTTErrors merr;

    ctx->connect_fd = -1;
    uev_io_stop(w);

    if (fd < 0) {
        CROSSLOGE("not connect_fd during socket callback");
        return;
    }

    rc = getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &sl);
    if (rc < 0) {
        CROSSLOG_ERRNO("getsockopt");
        return;
    }
    if (sl != sizeof(int)) {
        CROSSLOGE("SO_ERROR returned unsupported length: %zu", (size_t)sl);
        return;
    }

    if (err) {
        CROSSLOGE("connect failed: %d", err);
        return;
    }

    merr = on_connected(ctx, fd);
    if (merr != MQTT_OK) {
        CROSSLOGE("on_connected failed: %s", mqtt_error_str(merr));
        return;
    }
}

static enum MQTTErrors try_connect(struct mqtt_ctx *ctx, int fd) {
    int rc;
    struct sockaddr_in sa;

    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = htons(ctx->port);
    inet_addr_from_ip4addr(&sa.sin_addr, &ctx->addr.u_addr.ip4);

    /* connect to server */
    rc = connect(fd, (struct sockaddr *)&sa, sizeof(sa));
    if (rc < 0) {
        if (errno == EINPROGRESS) {
            rc = uev_io_init(ctx->uev, &ctx->w_sockfd, sockfd_cb, ctx, fd, UEV_WRITE | UEV_ERROR);
            if (rc) {
                CROSSLOG_ERRNO("uev_io_init");
                close(fd);
                return MQTT_ERROR_SOCKET_ERROR;
            }

            ctx->connect_fd = fd;
            return MQTT_OK;
        }

        /* let the mqtt timeout trigger, so we don't retry without any waiting time */
        CROSSLOG_ERRNO("connect");
        close(fd);
        return MQTT_ERROR_SOCKET_ERROR;
    }

    return on_connected(ctx, fd);
}

static enum MQTTErrors reconnect_client(struct mqtt_client *client, void **_pctx)
{
    struct mqtt_ctx *ctx = *_pctx;
    int rc;
    int fd;

    CROSSLOGI("(re)connect mqtt client");

    if (ctx->listener_registered) {
        usfs_listener_del(&ctx->listener);
        ctx->listener_registered = false;
    }

    /* Close the clients socket if this isn't the initial reconnect call */
    if (client->error != MQTT_ERROR_INITIAL_RECONNECT && client->socketfd >= 0) {
        close(client->socketfd);
        client->socketfd = -1;
    }

    /* Perform error handling here. */
    if (client->error != MQTT_ERROR_INITIAL_RECONNECT) {
        CROSSLOGE("reconnect_client: called while client was in error state \"%s\"", 
               mqtt_error_str(client->error)
        );
    }

    if (ctx->connect_fd >= 0) {
        uev_io_stop(&ctx->w_sockfd);
        close(ctx->connect_fd);
        ctx->connect_fd = -1;
    }

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        CROSSLOG_ERRNO("socket");
        return MQTT_ERROR_SOCKET_ERROR;
    }

    rc = fcntl(fd, F_GETFL);
    if (rc < 0) {
        CROSSLOG_ERRNO("fcntl:GETFL");
        close(fd);
        return MQTT_ERROR_SOCKET_ERROR;
    }

    rc = fcntl(fd, F_SETFL, rc | O_NONBLOCK);
    if (rc < 0) {
        CROSSLOG_ERRNO("fcntl:SETFL");
        close(fd);
        return MQTT_ERROR_SOCKET_ERROR;
    }

    return try_connect(ctx, fd);
}

static void fatal_error_callback(struct mqtt_client *client) {
    struct mqtt_ctx *ctx = client->userctx;

    CROSSLOGE("fatal mqtt client error: %s", mqtt_error_str(client->error));

    if (ctx->connect_fd >= 0) {
        uev_io_stop(&ctx->w_sockfd);
        close(ctx->connect_fd);
        ctx->connect_fd = -1;
    }

    if (ctx->listener_registered) {
        usfs_listener_del(&ctx->listener);
        ctx->listener_registered = false;
    }
}

static void on_usfs_enabled(void *_ctx, bool enabled) {
    struct mqtt_ctx *ctx = _ctx;

    send_enabled(ctx, enabled);
}

static void on_usfs_status(void *_ctx, uint8_t status) {
    struct mqtt_ctx *ctx = _ctx;

    send_imustatus(ctx, status);
}

static void on_usfs_samplerate(void *_ctx, unsigned int samplerate) {
    struct mqtt_ctx *ctx = _ctx;

    send_samplerate(ctx, samplerate);
}

void mdns_resolved_cb(struct mqtt_ctx *ctx) {
    enum MQTTErrors merr;

    CROSSLOGI("resolved: " IPSTR ":%u", IP2STR(&(ctx->addr.u_addr.ip4)), ctx->port);

    ctx->connect_fd = -1;
    ctx->listener.ctx = ctx;
    ctx->listener.enabled = on_usfs_enabled;
    ctx->listener.status = on_usfs_status;
    ctx->listener.samplerate = on_usfs_samplerate;

    merr = mqtt_init_reconnect(&ctx->client, ctx->uev, reconnect_client, ctx, publish_callback, fatal_error_callback);
    if (merr != MQTT_OK) {
        CROSSLOGE("mqtt_init_reconnect failed: %s", mqtt_error_str(merr));
        return;
    }
    ctx->client.userctx = ctx;
    ctx->client.publish_response_callback_state = ctx;

    merr = mqtt_start_reconnect(&ctx->client);
    if (merr != MQTT_OK) {
        CROSSLOGE("initial mqtt_start_reconnect failed: %s", mqtt_error_str(merr));
        return;
    }
}
