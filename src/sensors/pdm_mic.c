#include <zephyr.h>

#include <drivers/include/nrfx_pdm.h>
#include <nrf.h>
#include <nrfx_pdm.h>

#include <arm_math.h>

#include <date_time.h>

/*Logging*/
#include <logging/log.h>
LOG_MODULE_REGISTER(pdm_mic);

#include "pdm_mic.h"
#include "subsys/led.h"
// #include "utils/dspinst.h"


/* PDM -------------------------------------------------*/

int16_t pdm_idle[PDM_IDLE_BUFF_LEN];
pdm_buff_t pdm_buffers[PDM_BUFF_NUM];
volatile pdm_buff_t *recording_buff;
volatile pdm_buff_t *processing_buff;
volatile uint8_t active_buff = 0;

struct k_sem pdm_sem;

/*
	ISR workaround given by Nordic
*/
ISR_DIRECT_DECLARE(pdm_isr_handler)
{
	nrfx_pdm_irq_handler();
	ISR_DIRECT_PM(); /* PM done after servicing interrupt for best latency*/

	return 1; /* We should check if scheduling decision should be made */
}

/*
	Event handler for PDM input buffer
*/
void nrfx_pdm_event_handler(nrfx_pdm_evt_t const *const p_evt)
{
    if(p_evt->error != 0)
    {
        printk("nrfx_pdm_event_handler error: %i, released: %i, requested: %i\n", (int)p_evt->error, (int)p_evt->buffer_released, (int)p_evt->buffer_requested);
        return;
    }

    int64_t now = 0;
    date_time_now(&now);

    // pdm buffer is full - make it available for processing and move recording buff to next one
    if (p_evt->buffer_released != NULL)
    {   
        // .pdm_buff is first struct element so &pdm_buffers[active_buff].pdm_buff == &pdm_buffers[active_buff]
        // p_evt->buffer_released is pointing at the top half of the buffer when windowing - so we need to move the pointer down by PDM_BUFF_OVERLAP_LEN
        processing_buff = (pdm_buff_t *)(p_evt->buffer_released - PDM_BUFF_OVERLAP_LEN);
        processing_buff->status = PDM_BUFF_STATUS_IS_READY_FOR_PROCESSING;
        processing_buff->end_ts = now;

        // let the processing loop grab the sample
        k_sem_give(&pdm_sem);
        
        //printk("%lld: set p_buff[%i] time: %lld\n", unix_time_get(), processing_buff->index, processing_buff->end_ts - processing_buff->start_ts);

        active_buff = (active_buff + 1) % PDM_BUFF_NUM;
        recording_buff = &pdm_buffers[active_buff];
	}

	if (p_evt->buffer_requested) 
    {
        recording_buff->end_ts = 0;
        processing_buff->start_ts = now;
        recording_buff->status = PDM_BUFF_STATUS_IS_WRITING;
        
        //printk("%lld: set r_buff[%i]\n", unix_time_get(), recording_buff->index);
        
        // PDM_BUF_FFT_SIZE_PWR_OF_2 / 2 for windowing - this will only work for windows that require 50% overlap
        // always write to top half of buffer with half width of buffer - top samples were moved to beginning of buffer after window was applied in dba_calc()
        nrfx_pdm_buffer_set((int16_t *)&recording_buff->pdm_buff[PDM_BUFF_OVERLAP_LEN], PDM_BUFF_OVERLAP_LEN);
	}
}

void pdm_init_buffs(void)
{
    // clear buffer status flags
    for (int i = 0; i < PDM_BUFF_NUM; i++)
    {
        pdm_buffers[i].end_ts = 0;
        pdm_buffers[i].start_ts = 0;
        pdm_buffers[i].status = PDM_BUFF_STATUS_IS_READY_FOR_WRITING;
        pdm_buffers[i].index = i;   // debugging var to confirm dba results were done on the correct buffer
    }

    active_buff = 0;
    recording_buff = &pdm_buffers[active_buff];
    processing_buff = &pdm_buffers[active_buff+1];

    k_sem_init(&pdm_sem, 0, 1);
}

/*
	Initialization for PDM driver
*/
int pdmInit(void)
{
	LOG_INF("PDM init...");

    pdm_init_buffs();

	IRQ_DIRECT_CONNECT(PDM_IRQn, 0, pdm_isr_handler, 0);  

    static nrfx_pdm_config_t pdm_config = NRFX_PDM_DEFAULT_CONFIG(CLK_PIN, DIN_PIN);
	pdm_config.edge = NRF_PDM_EDGE_LEFTRISING;
    pdm_config.clock_freq = NRF_PDM_FREQ_1280K;
    pdm_config.ratio = NRF_PDM_RATIO_80X;   // 16.000kHz @ 1.280MHz clock, IMP34DT05 mic has 1.2MHz mimimum. Ratio64 gives only 1.032MHZ
    
	nrfx_err_t err = nrfx_pdm_init(&pdm_config, nrfx_pdm_event_handler);
	if (err != NRFX_SUCCESS) 
    {
        return err;
    }

	LOG_INF("PDM init done");
    
    return 0;
}

/*
	Start PDM recording
*/
void pdmStart(void)
{
    pdm_init_buffs();
	nrfx_pdm_start();

    LOG_INF("PDM started");
}

/*
	End PDM recording, call pdmReady() after stop
*/
void pdmStop(void)
{
    nrfx_pdm_stop();
}

pdm_buff_t *pdm_get_processing_buff(void)
{
    if (k_sem_take(&pdm_sem, K_MSEC(64)) == 0)
    {
        return (pdm_buff_t *)processing_buff;
    }
    
    return NULL;
}