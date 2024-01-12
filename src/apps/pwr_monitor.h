#ifndef PWR_MONITOR_H_
#define PWR_MONITOR_H_

#include <zephyr.h>
#include "apps.h"

struct tele_message
{
    int64_t ts;

    uint8_t power_status;

    uint16_t batt_mV;

    int8_t chrg_stat;
 
    apps_onboard_telem_t onboard_telem;
};

void app_pwr_monitor_init(void);
void pwr_chrg_pub_cb();

#endif /* PWR_MONITOR_H_ */