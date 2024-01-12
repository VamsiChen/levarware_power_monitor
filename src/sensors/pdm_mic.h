#ifndef MICROPHONE_SENSOR_H_
#define MICROPHONE_SENSOR_H_

/* 
    Variable && Define
*/
#define PDM_SAMPLE_RATE (16000.0)


#define PDM_BUF_FFT_SIZE_PWR_OF_2                   (1024) // 64ms
#define PDM_BUFF_FFT_NUM_FREQS                      (PDM_BUF_FFT_SIZE_PWR_OF_2  / 2) // number of frequencies in fft_data (excluding 0Hz)

#define PDM_BUFF_LEN                                (PDM_BUF_FFT_SIZE_PWR_OF_2)
#define PDM_BUFF_NUM                                (2) // 2 buffers for pdm to record while other is processing
#define PDM_BUFF_OVERLAP_LEN                        (PDM_BUFF_LEN / 2)

/*Define*/
#define  GPIO0                                      DT_LABEL(DT_NODELABEL(gpio0))
#define  CLK_PIN                                    30 						// D6 -> CLK
#define  DIN_PIN                                    31						// D7 -> DOUT

#define NRF_PDM_HAS_RATIO_CONFIG                    1

#define PDM_IDLE_BUFF_LEN                           1

#define PDM_BUFF_STATUS_IS_READY_FOR_WRITING        0   
#define PDM_BUFF_STATUS_IS_WRITING                  1
#define PDM_BUFF_STATUS_IS_READY_FOR_PROCESSING     2   
#define PDM_BUFF_STATUS_IS_PROCESSING               4   

typedef struct pdm_buff
{
    // MUST be first element for buff pointer logic
    int16_t pdm_buff[PDM_BUF_FFT_SIZE_PWR_OF_2];
    uint8_t index;
    uint8_t status;
    uint64_t start_ts;
    uint64_t end_ts;
} pdm_buff_t;

extern pdm_buff_t pdm_buffers[PDM_BUFF_NUM];

/* 
    PDM functions
*/
int pdmInit(void);
void pdmStart(void);
void pdmStop(void);
pdm_buff_t *pdm_get_processing_buff(void);

#endif /* MICROPHONE_SENSOR_H_ */