#ifndef DISTANCE_H_
#define DISTANCE_H_

#include <zephyr.h>
#include "apps.h"

#define APP_DISTANCE_STACK_SIZE       (2048*2)
#define APP_DISTANCE_PRIORITY         (3)

/* 
* Structure to hold distance data measurements
*/
struct distance_data
{
    int64_t ts;
    int32_t distance_mm;
    int8_t power_status;
    apps_onboard_telem_t onboard_telem;
};

void app_distance(void);
void distance_set_verbose_print_state(bool state, bool toggle);

#endif /* DISTANCE_H_ */