#ifndef MAIN_H
#define MAIN_H

#include <uev/uev.h>
#include <mqtt.h>
#include <stdint.h>
#include <sys/socket.h>
#include "list.h"

struct usfs_listener {
    struct list_node node;

    void *ctx;
    void (*enabled)(void *ctx, bool enabled);
    void (*status)(void *ctx, uint8_t status);
    void (*samplerate)(void *ctx, unsigned int rate);
};

struct mqtt_ctx {
    int connect_fd;
    uev_ctx_t *uev;

    uev_t w_resolved;
    uev_t w_pingtimer;
    uev_t w_acktimer;
    uev_t w_sockfd;

    ip_addr_t addr;
    uint16_t port;
    struct mqtt_client client;
    uint8_t sendbuf[2048];
    uint8_t recvbuf[1024];

    struct usfs_listener listener;
    bool listener_registered;
};

void init_usfs(uev_ctx_t *uev);
bool usfs_is_enabled(void);
void usfs_set_enabled(bool enabled);
uint8_t usfs_status(void);
unsigned int usfs_samplerate(void);
void usfs_listener_add(struct usfs_listener *listener);
void usfs_listener_del(struct usfs_listener *listener);

int init_mdns(uev_ctx_t *uev, struct mqtt_ctx *ctx);
void mdns_resolved_cb(struct mqtt_ctx *ctx);

int sdcard_init(void);
void sdcard_deinit(void);

#endif /* MAIN_H */
