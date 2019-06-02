#include "main.h"
#include <tcpip_adapter.h>

#define CROSSLOG_TAG "main"
#include <crosslog.h>

static void publish_callback(void **unused, struct mqtt_response_publish *published)
{
    CROSSLOGI("Received publish('%.*s'): %.*s", (int)published->topic_name_size, (const char *)published->topic_name,
        (int)published->application_message_size, (const char*) published->application_message);
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
    merr = mqtt_subscribe(&ctx->client, "test", 0);
    if (merr != MQTT_OK) {
        CROSSLOGE("mqtt_subscribe: %s", mqtt_error_str(merr));
        close(fd);
        return merr;
    }

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

        /* let the mqtt timeout rigger, so we don't retrying without any waiting time */
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
}

void mdns_resolved_cb(struct mqtt_ctx *ctx) {
    enum MQTTErrors merr;

    CROSSLOGI("resolved: " IPSTR ":%u", IP2STR(&(ctx->addr.u_addr.ip4)), ctx->port);

    ctx->connect_fd = -1;

    merr = mqtt_init_reconnect(&ctx->client, ctx->uev, reconnect_client, ctx, publish_callback, fatal_error_callback);
    if (merr != MQTT_OK) {
        CROSSLOGE("mqtt_init_reconnect failed: %s", mqtt_error_str(merr));
        return;
    }
    ctx->client.userctx = ctx;

    merr = mqtt_start_reconnect(&ctx->client);
    if (merr != MQTT_OK) {
        CROSSLOGE("initial mqtt_start_reconnect failed: %s", mqtt_error_str(merr));
        return;
    }
}
