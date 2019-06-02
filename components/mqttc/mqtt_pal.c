#include <mqtt.h>
#include <errno.h>
#include <sys/socket.h>

#define CROSSLOG_TAG "mqtt_pal"
#include <crosslog.h>

static void timer_cb(uev_t *w, void *_timer, int events) {
    mqtt_pal_timer_t *timer = _timer;
    timer->cb(timer->ctx);
}

int mqtt_pal_timer_init(mqtt_pal_ev_t *ev, mqtt_pal_timer_t *timer, mqtt_pal_timer_cb_t cb, void *ctx) {
    int rc;

    timer->cb = cb;
    timer->ctx = ctx;

    rc = uev_timer_init(ev, &timer->w, timer_cb, timer, 0, 0);
    if (rc) {
        CROSSLOG_ERRNO("uev_timer_init(0, 0)");
        return -1;
    }

    return 0;
}

int mqtt_pal_timer_set(mqtt_pal_timer_t *timer, size_t timeout, size_t period) {
    int rc;

    rc = uev_timer_set(&timer->w, timeout * 1000, period * 1000);
    if (rc) {
        CROSSLOG_ERRNO("uev_timer_set(%zu, %zu)", timeout, period);
        return -1;
    }

    return 0;
}

static void io_cb(uev_t *w, void *_io, int events) {
    mqtt_pal_io_t *io = _io;
    io->cb(io->ctx, events);
}

int mqtt_pal_io_init(mqtt_pal_ev_t *ev, mqtt_pal_io_t *io, mqtt_pal_io_cb_t cb, void *ctx) {
    io->ev = ev;
    io->cb = cb;
    io->ctx = ctx;
    io->sock = -1;

    return 0;
}

int mqtt_pal_io_set(mqtt_pal_io_t *io, mqtt_pal_socket_handle sock, size_t events) {
    int rc;

    if (io->sock == -1) {
        if (sock == -1)
            return 0;

        rc = uev_io_init(io->ev, &io->w, io_cb, io, sock, events);
    }
    else {
        if (sock == -1)
            rc = uev_io_stop(&io->w);
        else
            rc = uev_io_set(&io->w, sock, events);
    }

    if (rc) {
        CROSSLOG_ERRNO("mqtt_pal_io_set(%d, %zu), prevsock=%d", sock, events, io->sock);
        return -1;
    }

    io->sock = sock;
    return 0;
}

ssize_t mqtt_pal_send(mqtt_pal_socket_handle fd, const void* buf, size_t len, int flags) {
    size_t sent = 0;

    while (sent < len) {
        ssize_t tmp = send(fd, buf + sent, len - sent, flags);
        if (tmp < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                break;
            if (errno == EINTR)
                continue;

            CROSSLOG_ERRNO("send");
            return MQTT_ERROR_SOCKET_ERROR;
        }

        if (tmp == 0)
            return MQTT_ERROR_CONNECTION_CLOSED;

        sent += (size_t) tmp;
    }
    return sent;
}

ssize_t mqtt_pal_recv(mqtt_pal_socket_handle fd, void* buf, size_t bufsz, int flags) {
    const void const *start = buf;
    ssize_t rv;
    do {
        rv = recv(fd, buf, bufsz, flags);
        if (rv > 0) {
            /* successfully read bytes from the socket */
            buf += rv;
            bufsz -= rv;
        } else if (rv < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                break;

            if (errno != EINTR) {
                /* an error occurred that wasn't "nothing to read". */
                CROSSLOG_ERRNO("send");
                return MQTT_ERROR_SOCKET_ERROR;
            }
        }
        else {
            return MQTT_ERROR_CONNECTION_CLOSED;
        }
    } while (rv > 0);

    return buf - start;
}
