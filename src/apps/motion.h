#ifndef AUDIO_H_
#define AUIDO_H_

#include <zephyr.h>
#include "apps.h"

#define APP_MOTION_STACK_SIZE       (2048)
#define APP_MOTION_PRIORITY         (2)


/*! Number of Accel frames to be extracted from FIFO */
#define BMA2_FIFO_EXTRACTED_DATA_FRAME_COUNT  UINT8_C(32)


typedef struct motion_data
{
    int64_t ts;
    int32_t distance_mm;
    apps_onboard_telem_t onboard_telem;
} motion_data_t;

void app_motion(void);
void distance_set_verbose_print_state(bool state, bool toggle);

#endif /* AUIDO_H_ */