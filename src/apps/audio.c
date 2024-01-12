#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <date_time.h>
#include <sys/reboot.h>

/*Logging*/
#include <logging/log.h>
LOG_MODULE_REGISTER(app_audio);

#include <arm_math.h>
#include <arm_const_structs.h>

#include "apps.h"
#include "audio.h"
#include "cloud.h"
#include "device_config/device_config.h"
#include "sensors/pdm_mic.h"
#include "utils/acurve.h"
#include "utils/data_windows.h"

static q31_t window_buff[PDM_BUF_FFT_SIZE_PWR_OF_2];

static arm_rfft_fast_instance_f32 S_f32;
static float fft_input_buffer[PDM_BUF_FFT_SIZE_PWR_OF_2];
static float fft_output_buffer[PDM_BUF_FFT_SIZE_PWR_OF_2];
static float fft_mag_buffer[PDM_BUFF_FFT_NUM_FREQS];

// const int16_t test_audio[1024] = {
// 208,210,208,204,199,204,208,204,211,206,213,211,207,208,208,201,196,195,192,194,195,199,189,190,192,187,192,191,190,188,188,189,185,187,182,183,183,185,187,192,193,204,201,198,192,189,178,182,184,186,192,188,192,195,193,184,184,182,184,189,193,195,199,200,197,189,190,194,189,197,194,194,199,203,213,217,219,214,209,208,211,217,222,225,227,224,217,215,208,205,201,207,202,207,213,221,228,221,222,214,214,216,213,220,227,236,238,241,238,233,229,221,218,206,215,221,223,228,230,230,226,216,211,204,210,216,230,242,232,237,223,221,210,200,171,142,109,83,68,49,33,22,18,26,43,68,87,96,94,80,59,38,15,-9,-14,-37,-48,-58,-56,-46,-20,-22,-19,-22,-25,-21,-7,16,36,52,66,74,92,105,118,89,59,57,64,87,119,160,178,205,231,245,253,283,288,296,298,301,293,293,301,303,309,327,319,320,377,442,442,420,411,495,539,559,579,578,321,189,-49,112,82,103,-34,-84,-99,-40,149,226,319,301,306,170,105,-5,-117,-228,-264,-283,-295,-281,-261,-243,-212,-58,43,140,129,127,118,184,262,322,374,431,474,488,492,462,437,424,454,500,544,557,573,606,658,720,745,729,687,625,571,511,472,406,333,280,253,227,196,151,125,108,116,135,166,167,188,206,220,251,244,235,205,200,159,132,109,79,59,56,84,83,83,80,77,100,144,178,179,184,200,220,235,251,254,237,217,206,209,207,214,225,230,251,282,306,327,345,371,383,396,412,404,397,411,405,398,384,374,358,332,320,299,274,256,243,247,248,257,257,252,255,275,309,327,347,344,362,366,394,401,389,390,395,370,358,341,302,262,227,225,184,185,160,149,141,168,196,194,167,199,206,240,262,289,268,262,290,292,284,291,313,278,280,318,328,333,333,344,355,355,387,370,338,321,346,343,322,334,296,274,273,290,271,265,253,247,234,246,254,250,247,257,270,270,274,262,251,240,248,240,231,218,212,205,209,213,210,199,202,203,220,227,228,230,236,234,266,283,285,253,241,236,233,237,235,244,226,248,250,269,272,269,282,292,311,308,323,316,327,341,336,326,324,312,300,302,295,297,298,291,280,293,286,281,285,275,269,275,287,286,292,291,283,278,290,286,278,274,271,267,279,287,293,300,303,313,310,305,304,308,307,316,333,334,340,345,333,319,322,328,314,316,315,318,319,317,318,313,320,330,338,351,353,355,352,338,343,351,364,364,356,355,367,379,387,382,373,374,375,366,351,362,358,338,322,331,320,322,318,312,299,310,318,327,329,321,317,309,322,316,315,313,322,314,316,314,320,310,298,295,287,297,304,304,296,314,310,313,312,315,312,313,315,311,325,337,345,369,376,379,385,382,392,398,405,406,405,408,400,392,390,388,376,371,367,357,362,355,366,359,364,363,369,361,363,363,368,370,367,368,360,362,353,357,357,359,349,344,346,341,348,357,359,354,359,361,365,364,358,353,358,357,353,352,351,357,352,352,347,344,350,354,355,344,347,339,342,343,346,345,338,344,329,340,325,344,311,321,309,292,289,283,311,274,278,293,293,266,274,273,289,273,298,283,297,268,282,285,279,302,300,312,306,331,319,321,321,308,337,323,337,337,332,330,354,345,348,359,357,347,336,343,360,373,382,383,376,395,393,395,374,379,385,369,377,385,402,351,401,355,376,332,363,338,340,335,332,325,320,341,337,320,337,336,331,323,318,330,329,331,324,302,323,305,336,316,336,316,336,318,345,336,345,351,336,335,342,346,359,362,353,343,344,361,352,366,336,356,318,361,357,376,365,361,393,371,388,359,395,375,373,357,349,359,351,360,346,337,357,343,337,332,344,359,342,369,344,368,352,358,363,342,364,337,354,329,319,311,306,324,293,311,307,323,334,350,346,340,357,360,351,364,343,358,338,369,345,363,338,358,356,351,364,367,347,348,354,363,365,381,371,351,361,346,381,341,360,333,342,334,335,333,322,352,342,368,355,363,357,363,361,365,370,374,352,346,327,333,339,343,336,321,331,318,315,307,329,322,326,338,342,330,337,348,341,343,362,348,341,342,342,333,331,354,341,341,343,365,354,347,367,343,374,352,363,353,349,356,346,350,341,348,330,327,326,328,334,333,334,333,333,337,330,333,347,352,355,349,342,331,327,332,332,336,325,328,322,319,317,327,330,333,348,341,343,344,349,347,346,352,351,350,348,349,346,348,345,348,349,344,350,353,359,362,370,370,374,373,375,366,371,365,364,364,364,364,359,361,357,354,356,366,374,374,371,373,374,374,372,372,369,376,369,372,370,374,367,359,360,359,361,364,368,364,371,370,373
// };

// goal is to get an LEQ value every minute - we get a dba value every 64ms so 1 minute is 937.5 dba values to calculate leq from
#define AUDIO_LAEQ_SAMPLES_1SEC         31      // 0.992s 
#define AUDIO_LAEQ_SAMPLES_10SEC        313     // 10.016s
#define AUDIO_LAEQ_SAMPLES_30SEC        938     // 30.016s
#define AUDIO_LAEQ_SAMPLES_60SEC        1875    // 60.0s
#define AUDIO_LAEQ_SAMPLES              AUDIO_LAEQ_SAMPLES_1SEC
#define AOP_CONSTANT                    95.4073    // for dba calculation aop constant = aop - (log(number of samples of raw data)/10). aop = 122.5

#define LAEQ_REBOOT_MAX                 10      // Nui,ber  of LAeq overflows permitted before reboot of system - bandaid fix for field issues. 
static float laeq_buffer[AUDIO_LAEQ_SAMPLES] = {0}; 
static int32_t dba_buffer_index = 0;

#define MAX_LAEQ_PUB_INTERVAL_SAMPLES   120     // up to ~2 minutes
static acoustic_data_t laeq_over_pub_period[MAX_LAEQ_PUB_INTERVAL_SAMPLES];   
static int32_t laeq_over_pub_period_index = 0;

#define MAX_LAEQ_PUB_MSGQ_OVERFLOW_TIME_MS   (60 * MSEC_PER_SEC)

// counter to track number  of laeq buffer overflows before performing reboot
static int reboot_counter = 0;

// static define single hs buffer for this app
apps_hs_telem_data_t hs_telem_data;
static struct k_msgq acoustic_msgq;

static q15_t data_pdm_buff[PDM_BUF_FFT_SIZE_PWR_OF_2];
static float a_weighted_mag;

// static struct k_work telemety_work;
static struct k_work_delayable telemety_work;
// static struct k_work_sync work_sync;

typedef struct laeq_work_info {
    struct k_work work;
    struct k_sem sem;
    acoustic_data_t data;
    float laeq_buffer[AUDIO_LAEQ_SAMPLES];
    int32_t laeq_pub_duration_sum_ms;
} laeq_work_t;

laeq_work_t laeq_work = {
    .data.duration_ms = 0,
    .data.instant_dba = 0,
    .data.laeq_db = 0,
    .data.max_dba = 0,
    .data.ts = 0,
    .laeq_buffer = { 0.0 },
    .laeq_pub_duration_sum_ms = 0
};

struct k_thread app_audio_thread_data;
K_THREAD_STACK_DEFINE(app_audio_stack_area, APP_AUDIO_STACK_SIZE);


void app_audio_entry_point(void *p1, int unused2, int32_t unused3);
void app_audio_telemetry_work_fn(struct k_work *work);
void laeq_work_fn(struct k_work *item);
float calc_laeq(float *p_dba_buff, int num_samples);
float calc_laeq_pub_period(acoustic_data_t *ret_val, int32_t count);

static pdm_buff_t *mic_process_pdm(void);

bool do_mic_listen = true;
bool is_mic_listening = false;
bool print_dba = false;


static void work_init(void)
{
    k_work_init_delayable(&telemety_work, app_audio_telemetry_work_fn);

    k_sem_init(&laeq_work.sem, 1, 1);
    k_work_init(&laeq_work.work, laeq_work_fn);
}

void app_audio(void)
{
    k_thread_create(&app_audio_thread_data, 
                    app_audio_stack_area, 
                    K_THREAD_STACK_SIZEOF(app_audio_stack_area), 
                    (k_thread_entry_t)app_audio_entry_point, 
                    NULL,
                    NULL,
                    NULL,
                    APP_AUDIO_PRIORITY,
                    0,
                    K_MSEC(5000)); // give cloud thread a headstart
    k_thread_name_set(&app_audio_thread_data, "app_audio");
}

void app_audio_entry_point(void *p1, int unused2, int32_t unused3)
{
    int err = 0;

    LOG_INF("Audio App thread started");

    // TODO: Initialize queue here, struct {ts, sound_pressure_pa}
    char __aligned(4) acoustic_data_msgq[10 * sizeof(acoustic_data_t)];
    k_msgq_init(&acoustic_msgq, acoustic_data_msgq, sizeof(acoustic_data_t), 10);

    // init fft
    arm_rfft_fast_init_f32(&S_f32, PDM_BUF_FFT_SIZE_PWR_OF_2);

    err = pdmInit();
    if (err)
    {
        LOG_ERR("failed to init PDM: %d", err);
    }

    work_init();
    
    dba_buffer_index = 0;
    laeq_over_pub_period_index = 0;
    laeq_work.laeq_pub_duration_sum_ms = 0;
    
    LOG_INF("Starting PDM...");
    pdmStart();

    // +4 seconds for startup of rest of system and daq queue (experementally determined)
    k_work_schedule(&telemety_work, K_SECONDS(device_config_get_int16(DEV_CONFIG_PUB_INTERVAL_S) + 4));

    while (1)
    {
        // wait for next pdm buff and perform fft
        // will start the laeq work thread when buffer full
        if (mic_process_pdm() == NULL)
        {
            // unable to get pdm buffer... unlikey to ever happen, but if it does reset pdm and try again
            LOG_INF("Unable to get pdm buff, restarting pdm...");
            pdmStop();

            // short delay
            k_msleep(250);
            
            pdmStart();
        }
    }
}

// this will be called every 32ms with 1024 fft and 8kHz PDM
static pdm_buff_t *mic_process_pdm(void)
{
    static int64_t overflow_ts = 0;

    pdm_buff_t *b = pdm_get_processing_buff();

    arm_copy_q15((q15_t *)&b->pdm_buff[PDM_BUFF_OVERLAP_LEN], (q15_t *)&data_pdm_buff[PDM_BUFF_OVERLAP_LEN], PDM_BUFF_OVERLAP_LEN);

    
    if (b != NULL)
    {
        int64_t now = k_uptime_get();
        
        float dba = calc_dba(data_pdm_buff);
        
        if (print_dba)
        {
            printk("%lld: pdm_buff[%d] in %ims -> %0.2f dB(A)\n", now,  b->index, (uint32_t)(k_uptime_get() - now), dba);
        }

        laeq_buffer[dba_buffer_index++] = dba;
        
        if (dba_buffer_index >= AUDIO_LAEQ_SAMPLES)
        {
            dba_buffer_index = 0;

            // we need to send further processing to workers to prevent missing the next pdm sample
            // single worker context - so wait a bit for the previous to finish - should use thread status instead of a sem, I think
            if (k_sem_take(&laeq_work.sem, K_MSEC(500)) == 0)
            {
                // always reset overflow reboot counter if we have the semaphore
                overflow_ts = 0;

                // copy buffer to work context so we can continue on to next sample while processing and publishing are running async
                for (int i = 0; i < AUDIO_LAEQ_SAMPLES; i++)
                {
                    laeq_work.laeq_buffer[i] = laeq_buffer[i];
                }
                k_work_submit(&laeq_work.work);
            }
            else
            {
                // This should not happen - if it does, it means the laeq calculations are taking more time than the laeq_buffer allows 
                LOG_ERR("laeq_work overflow!");
                reboot_counter++;               // track number of buffer overflows
                LOG_INF("LAeq overflow reboot counter %d", reboot_counter);

                if(reboot_counter == LAEQ_REBOOT_MAX)       // if reached terminal count, reboot
                    sys_reboot(0);
                
                // just in case something went wrong in worker - cancel and force-reset the semaphore
                k_work_cancel(&laeq_work.work);
                k_sem_give(&laeq_work.sem);

                // capture rising edge of overflow event with static var to track reboot timeout
                if (overflow_ts == 0)
                {
                    overflow_ts = k_uptime_get();
                }
                else if (k_uptime_get() - overflow_ts > MAX_LAEQ_PUB_MSGQ_OVERFLOW_TIME_MS)
                {
                    // if this happens, something has gone wrong - try a reboot
                    sys_reboot(0);
                }
            }
        }
    }

    return b;
}

// called after every dba_buffer fill interval - default ~1 second
// non-optimized so can take ~500ms - 700ms to complete

void laeq_work_fn(struct k_work *item)
{   
    float max_dba = 0;
    int32_t max_index = 0;
    int64_t now;
    
    date_time_now(&now);

    struct laeq_work_info *ctx = CONTAINER_OF(item, struct laeq_work_info, work);

    if (ctx->data.ts == 0)
    {
        // first time through has no usable data - update ts and discard sample
        ctx->data.ts = now;
        k_sem_give(&ctx->sem);
        return;
    }

    // get last sample - TODO: instant_dba may not be useful
    ctx->data.instant_dba = laeq_buffer[AUDIO_LAEQ_SAMPLES - 1];

    // actual duration of buffer is new ts - previous ts 
    ctx->data.duration_ms = (now >= ctx->data.ts) ? now - ctx->data.ts : 0;
    ctx->data.ts = now;
    
    arm_max_f32(&ctx->laeq_buffer[0], AUDIO_LAEQ_SAMPLES, &max_dba, &max_index);
    ctx->data.max_dba = max_dba;

    // can't find a fast pow(10, x) so don't calc_laeq too often
    ctx->data.laeq_db = calc_laeq(&ctx->laeq_buffer[0], AUDIO_LAEQ_SAMPLES);
    // Enable/Disable printk 
    // printk("%lld (%ims): LAeq = %0.2f dB, Max = %0.2f dB(A)\n", now, ctx->data.duration_ms, ctx->data.laeq_db, max_dba);
    
    // copy into laeq buffer for publish period
    laeq_over_pub_period[laeq_over_pub_period_index].ts = ctx->data.ts; 
    laeq_over_pub_period[laeq_over_pub_period_index].duration_ms = ctx->data.duration_ms;
    laeq_over_pub_period[laeq_over_pub_period_index].instant_dba = ctx->data.instant_dba;
    laeq_over_pub_period[laeq_over_pub_period_index].laeq_db = ctx->data.laeq_db;
    laeq_over_pub_period[laeq_over_pub_period_index].max_dba = ctx->data.max_dba; 
    laeq_over_pub_period_index++;

    // track total sum of time in pub array
    ctx->laeq_pub_duration_sum_ms += ctx->data.duration_ms;

    // do we have enough for a publish period?
    if ((ctx->laeq_pub_duration_sum_ms >= device_config_get_int16(DEV_CONFIG_PUB_INTERVAL_S) * MSEC_PER_SEC)
        || laeq_over_pub_period_index == MAX_LAEQ_PUB_INTERVAL_SAMPLES)
    {
        // don't re-use data_msgq, it needs the ts from previous to calc duration
        acoustic_data_t laeq_pub = { 
            .duration_ms = ctx->laeq_pub_duration_sum_ms, 
            .max_dba = 0, 
            .ts = 0, 
            .laeq_db = 0, 
            .instant_dba = 0
        };

        calc_laeq_pub_period(&laeq_pub, laeq_over_pub_period_index);
        
        // if DAQ interval is less than MIN PUB interval, it is considered high speed data and uses the 
        // hs_telem_data samples array to send multiple measurements in a single publish
        // each app or feature will handle hs data differently - audio is streaming so there will always be
        // hs data available with every publish if so configured, 
        // therefore the hs data struct is contained within the acoustic data struct allowing for 
        // sending at the same time as the standard telemetry but to the hs telem topic
        // Other apps may prefer an async approach directly to the hs telem topic
        if (device_config_get_int16(DEV_CONFIG_DAQ_INTERVAL_S) < PUB_INTERVAL_MINIMUM_S)
        {
            hs_telem_data.ts = laeq_pub.ts;
            hs_telem_data.measure_name = "laeq_db";
            hs_telem_data.sample_duration_ms = 0;
            
            hs_telem_data.num_samples = 0;
            for (int i = 0; i < laeq_over_pub_period_index && i < APPS_HS_TELEM_MAX_SAMPLES; i++)
            {
                hs_telem_data.samples.float_arr[i] = laeq_over_pub_period[i].laeq_db;
                hs_telem_data.sample_duration_ms += laeq_over_pub_period[i].duration_ms;
                hs_telem_data.num_samples++;
            }

            if (hs_telem_data.num_samples > 0)
            {
                // get the average duration of each sample in ms
                hs_telem_data.sample_duration_ms /= hs_telem_data.num_samples; 

                // send to high speed telemetry handler in apps.c
                apps_hs_send_data(&hs_telem_data);
            }     
        }
        
        laeq_over_pub_period_index = 0;
        ctx->laeq_pub_duration_sum_ms = 0;
        
        // this queue is read in the telemetry worker which then publishes it to the cloud
        if (k_msgq_put(&acoustic_msgq, &laeq_pub, K_NO_WAIT) != 0)
        {
            LOG_WRN("acoustic msgq is FULL");
        }
    }

    k_sem_give(&ctx->sem);
}

// self scheduling work function
void app_audio_telemetry_work_fn(struct k_work *work)
{
    acoustic_data_t data;
    int64_t start_ts = k_uptime_get();
    int32_t sleep_time_s = device_config_get_int16(DEV_CONFIG_PUB_INTERVAL_S);

    // process all data in queue but don't wait if its empty
    while (k_msgq_get(&acoustic_msgq, &data, K_NO_WAIT) == 0)
    {
        // audio is intended to be streamed, therefore we can get away with having onboard telem collected here
        // the worker function caller needs to run to completion asap so no time for onboard telem collection
        apps_get_onboard_telemetry(&data.onboard_telem);
        apps_telemetry_send((void *)&data, AUDIO);   
    }

    int32_t delta_s = (int32_t)(k_uptime_get() - start_ts) / MSEC_PER_SEC;
    sleep_time_s -= delta_s;

    // check for unusually long work times - for now, anything that takes longer than the publish time is an error
    // TODO: there must is a better max time than simply a fraction of the pub interval - perhaps a new config define for max telemetry work time in seconds?
    if (sleep_time_s <= 0)
    {
        // hmmm, what do we have here? corner case where our work took longer than our pub interval... 
        // this is an error state -> flush queue and sleep for max time instead
        sleep_time_s = device_config_get_int16(DEV_CONFIG_PUB_INTERVAL_S);
        k_msgq_purge(&acoustic_msgq);
        LOG_ERR("Telemetry work overflow -> telemetry msgq purged, sleep full pub interval");
    }    
    LOG_DBG("Next telemetry pub in %d seconds", sleep_time_s);

    // pushed item from LAEQ to cloud, if we had a laeq_work overflow, reset the count and log it
    if (reboot_counter != 0)
    {
        reboot_counter = 0;     // reset reboot counter for bandaid fix
        LOG_INF("LAeq overflow counter reset");
    }

k_work_schedule(&telemety_work, K_SECONDS(sleep_time_s));
}

//////// calculation methods

float calc_dba(int16_t *raw_data)
{   
    float dba = 0.0;

    // apply the hanning window function to raw _data.
    for(int i = 0; i < PDM_BUF_FFT_SIZE_PWR_OF_2; i++)
    {
        // this multiplication produce 32 bit array as its multiplying 2 x 16 bit arrays
        window_buff[i] = raw_data[i] * AudioWindowHanning1024[i];

    }

    // Converting the 32 bit int array to 32 bit float array as these values compute in floats.
    arm_q31_to_float(&window_buff[0], &fft_input_buffer[0], PDM_BUF_FFT_SIZE_PWR_OF_2);
  
    
    // Done with raw_data array - move current PDM_BUFF_OVERLAP_LEN samples to front of buffer for window overlap
    arm_copy_q15((q15_t *)&raw_data[PDM_BUFF_OVERLAP_LEN], (q15_t *)&raw_data[0], PDM_BUFF_OVERLAP_LEN);

    // Using 32 bit float array and performing FFT on it.
    arm_rfft_fast_f32(&S_f32, &fft_input_buffer[0], &fft_output_buffer[0], 0);

    //taking magnitude of the complex values obtained in above step using FFT. So array size is reduced to half of the orginal FFT array
    arm_cmplx_mag_f32(&fft_output_buffer[0], &fft_mag_buffer[0], PDM_BUFF_FFT_NUM_FREQS);

    float SUM_SQ_A_WEIGHTED_MAG = 0.0;
    
    for (int i = 0; i < PDM_BUFF_FFT_NUM_FREQS; i++)
    {   
        // Doubling the magnitude buffer becuase FFT produces results of only half wave of FFT as complete wave is identical and this saves computation time.
        fft_mag_buffer[i] = 2 * fft_mag_buffer[i];  

        //a weighting the fft magnitude values 
        a_weighted_mag = (fft_mag_buffer[i] * a_weighting_curve[i]) / PDM_BUFF_FFT_NUM_FREQS;
        
        // squaring the a_weighted_magnitude and then suming it up in previous value
        SUM_SQ_A_WEIGHTED_MAG += (a_weighted_mag * a_weighted_mag);
    }  
    
    dba = 10 * log10(SUM_SQ_A_WEIGHTED_MAG) + AOP_CONSTANT ;   // Formula for dba calculation

    return dba;
}

float calc_laeq(float *p_dba_buff, int num_samples)
{
    // https://www.cirrusresearch.co.uk/blog/2013/01/noise-data-averaging-how-do-i-average-noise-measurements/
    float sum = 0;

    //arm_power_f32(&dba_buffer[0], PDM_BUF_FFT_SIZE_PWR_OF_2, &sum);
    
    for (int i = 0; i < num_samples; i++)
    {
        sum += pow(10, p_dba_buff[i]/10); //times[i] * 10**(dbas[i]/10);
    }

    float laeq = 10 * log10(sum/num_samples); //10 * np.log10(1/T * sum);

    return laeq;
}

float calc_laeq_pub_period(acoustic_data_t *ret_val, int32_t count)
{
    float sum = 0;

    // the timestamp of the laeq over the pub period is the time of the first sample
    ret_val->ts = laeq_over_pub_period[0].ts;
    ret_val->max_dba = 0;
    ret_val->duration_ms = 0;

    // always calculate the whole array
    for (int i = 0; i < count; i++)
    {
        sum += pow(10, laeq_over_pub_period[i].laeq_db/10); //times[i] * 10**(dbas[i]/10);
        
        // updated max in array and sum of durations
        ret_val->max_dba = laeq_over_pub_period[i].max_dba > ret_val->max_dba ? laeq_over_pub_period[i].max_dba : ret_val->max_dba; 
        ret_val->duration_ms += laeq_over_pub_period[i].duration_ms;
    }

    float laeq = 10 * log10(sum/count); //10 * np.log10(1/T * sum);
    
    ret_val->laeq_db = laeq;
    if (ret_val->duration_ms < 0) 
    {
        ret_val->duration_ms = 0;
    }
    
    LOG_DBG("calc_laeq_pub_period: ts = %lld, duration_ms = %i, max_dba = %0.2f, laeq_db = %0.2f", ret_val->ts, ret_val->duration_ms, ret_val->max_dba, ret_val->laeq_db);

    return laeq;
}

void audio_set_or_toggle_print_dba_state(bool state, bool toggle)
{
   print_dba = toggle ? (print_dba ^ 1) : state;
}
