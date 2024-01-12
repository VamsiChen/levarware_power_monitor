#ifndef CLOUD_H_
#define CLOUD_H_

#include <zephyr.h>
#include "apps.h"
#include "device_config/device_config.h"

int cloud_start(void);
void cloud_send_json_str(char *topic, char *json_str);
void lte_connection_change(bool lte_up);

struct shadow_notification_blk {
    struct _snode node;             // linked list node to place on notification chain
    void (*notification_callback_fn) (void *userp); // shadow notification callback function pointer
    void *userp;                    // callers parameter, callback function will be called with this pointer
};


void cloud_register_shadow_change(struct shadow_notification_blk *cblk, void (*notification_callback_fn)(void *userp), void *userp);


#endif // CLOUD_H_