/*
 * Copyright (c) 2022 Reliance Foundry Co. Ltd.
 *
 */

#ifndef POWER_MGR_H_
#define POWER_MGR_H_

#include <zephyr.h>

#include "power_reg.h"

#define POWER_MGR_UPTIME_TIMER_S                (1800)  // 30 minutes

typedef enum power_state_t
{
    POWER_STATE_AWAKE = 0,
    POWER_STATE_SLEEP,
    POWER_STATE_UNKNOWN,
    POWER_STATE_NUM,
    POWER_STATE_INVALID
} power_state_t;

extern struct k_sem wakeup_sem;

void power_mgr_init_device_state(void);
void power_mgr_sleep_timer_cb(struct k_timer *t);
int power_mgr_set_device_state(power_state_t state);

#endif /*POWER_MGR_H_*/