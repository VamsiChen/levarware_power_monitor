#ifndef AUDIO_H_
#define AUIDO_H_

#include <zephyr.h>
#include <apps.h>

#define APP_AUDIO_STACK_SIZE           (2048)
#define APP_AUDIO_PRIORITY             (2)
#define MAX_LAEQ_PUB_INTERVAL_SAMPLES  120

#define PDM_BUF_FFT_SIZE_PWR_OF_2                   (1024) // 64ms


//new PDM buffer for storing data - swatej
typedef struct acoustic_data_t
{
    int64_t ts;
    int32_t duration_ms;
    apps_onboard_telem_t onboard_telem;
    float instant_dba;
    float max_dba;
    float laeq_db;
} acoustic_data_t;

extern struct k_msgq mic_raw_msgq;

void app_audio(void);
void audio_set_or_toggle_print_dba_state(bool state, bool toggle);

float calc_dba(int16_t *raw_data);
float calc_laeq(float *p_dba_buff, int num_samples);


#endif /* AUIDO_H_ */