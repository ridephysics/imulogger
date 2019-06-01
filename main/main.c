#include <uev/uev.h>

#define CROSSLOG_TAG "main"
#include <crosslog.h>

static uev_ctx_t _uev;
static uev_ctx_t * const uev = &_uev;

void app_main(void)
{
    int rc;

    CROSSLOGI("Hello world!");

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
