#ifndef MAIN_H
#define MAIN_H

#include <uev/uev.h>
#include <mqtt.h>
#include <stdint.h>
#include <sys/socket.h>

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
};

void init_usfs(void);
int init_mdns(uev_ctx_t *uev, struct mqtt_ctx *ctx);
void mdns_resolved_cb(struct mqtt_ctx *ctx);

#endif /* MAIN_H */
