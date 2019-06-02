#ifndef MQTT_PAL_H
#define MQTT_PAL_H

#include <limits.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <arpa/inet.h>
#include <uev/uev.h>

#define MQTT_PAL_HTONS(s) htons(s)
#define MQTT_PAL_NTOHS(s) ntohs(s)

#define MQTT_PAL_TIME() time(NULL)

typedef time_t mqtt_pal_time_t;
typedef void* mqtt_pal_mutex_t;

#define MQTT_PAL_MUTEX_INIT(mtx_ptr) do { *(mtx_ptr) = NULL; } while(0)
#define MQTT_PAL_MUTEX_LOCK(mtx_ptr) (void)(mtx_ptr)
#define MQTT_PAL_MUTEX_UNLOCK(mtx_ptr) (void)(mtx_ptr)

typedef int mqtt_pal_socket_handle;
typedef uev_ctx_t mqtt_pal_ev_t;
typedef void (*mqtt_pal_timer_cb_t)(void*);
typedef void (*mqtt_pal_io_cb_t)(void*, size_t events);

enum {
    MQTT_PAL_IO_ERROR = UEV_ERROR,
    MQTT_PAL_IO_READ = UEV_READ,
    MQTT_PAL_IO_WRITE = UEV_WRITE,
};

typedef struct {
    uev_t w;
    mqtt_pal_timer_cb_t cb;
    void *ctx;
} mqtt_pal_timer_t;

typedef struct {
    mqtt_pal_ev_t *ev;
    mqtt_pal_io_cb_t cb;
    void *ctx;

    mqtt_pal_socket_handle sock;
    uev_t w;
} mqtt_pal_io_t;

int mqtt_pal_timer_init(mqtt_pal_ev_t *ev, mqtt_pal_timer_t *timer, mqtt_pal_timer_cb_t cb, void *ctx);
int mqtt_pal_timer_set(mqtt_pal_timer_t *timer, size_t timeout, size_t period);

int mqtt_pal_io_init(mqtt_pal_ev_t *ev, mqtt_pal_io_t *io, mqtt_pal_io_cb_t cb, void *ctx);
int mqtt_pal_io_set(mqtt_pal_io_t *io, mqtt_pal_socket_handle sock, size_t events);

ssize_t mqtt_pal_send(mqtt_pal_socket_handle fd, const void* buf, size_t len, int flags);
ssize_t mqtt_pal_recv(mqtt_pal_socket_handle fd, void* buf, size_t bufsz, int flags);

#endif /* MQTT_PAL_H */
