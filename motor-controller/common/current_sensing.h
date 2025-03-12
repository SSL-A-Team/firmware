/**
 * @file setup.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-04-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#define V_DDA 3000.0f // mV
#define V_ADC_SCALE V_DDA / 4095.0f

// From A.7.16 of RM0091
#define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7C2))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))

#define VDD_CALIB ((uint16_t) (3300))
#define VDD_APPLI ((uint16_t) (V_DDA/10))

typedef enum {
  CS_MODE_POLLING,
  CS_MODE_DMA,
  CS_MODE_TIMER_DMA
} CS_Mode_t;

typedef enum
{
  CS_OK       = 0x00U,
  CS_ERROR    = 0x01U,
  CS_BUSY     = 0x02U,
  CS_TIMEOUT  = 0x03U
} CS_Status_t;

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))


// ADC resolutions
#define ADC_RESOLUTION_12B (0x00000000U)                       /*!< ADC resolution 12 bits */
#define ADC_RESOLUTION_10B (ADC_CFGR1_RES_0)                   /*!< ADC resolution 10 bits */
#define ADC_RESOLUTION_8B  (ADC_CFGR1_RES_1)                   /*!< ADC resolution  8 bits */
#define ADC_RESOLUTION_6B  (ADC_CFGR1_RES_1 | ADC_CFGR1_RES_0) /*!< ADC resolution  6 bits */

// ADC Data alignment
#define ADC_DATA_ALIGN_RIGHT (0x00000000U)                      /*!< ADC conversion data alignment: right aligned (alignment on data register LSB bit 0)*/
#define ADC_DATA_ALIGN_LEFT  (ADC_CFGR1_ALIGN)                  /*!< ADC conversion data alignment: left aligned (aligment on data register MSB bit 15)*/

// ADC Low power mode
#define ADC_LP_MODE_NONE (0x00000000U)                          /*!< No ADC low power mode activated */

// ADC Trigger mode
#define ADC_REG_TRIG_EXT_EDGE_DEFAULT   (ADC_CFGR1_EXTEN_1) // falling edge only
#define ADC_REG_TRIG_SOFTWARE           (0x00000000U)                                                             /*!< ADC group regular conversion trigger internal: SW start. */
#define ADC_REG_TRIG_EXT_TIM1_TRGO      (ADC_REG_TRIG_EXT_EDGE_DEFAULT)                                           /*!< ADC group regular conversion trigger from external IP: TIM1 TRGO. Trigger edge set to rising edge (default setting). */
#define ADC_REG_TRIG_EXT_TIM1_CH4       (ADC_CFGR1_EXTSEL_0 | ADC_REG_TRIG_EXT_EDGE_DEFAULT)                      /*!< ADC group regular conversion trigger from external IP: TIM1 channel 4 event (capture compare: input capture or output capture). Trigger edge set to rising edge (default setting). */
#define ADC_REG_TRIG_EXT_TIM2_TRGO      (ADC_CFGR1_EXTSEL_1 | ADC_REG_TRIG_EXT_EDGE_DEFAULT)                      /*!< ADC group regular conversion trigger from external IP: TIM2 TRGO. Trigger edge set to rising edge (default setting). */
#define ADC_REG_TRIG_EXT_TIM3_TRGO      (ADC_CFGR1_EXTSEL_1 | ADC_CFGR1_EXTSEL_0 | ADC_REG_TRIG_EXT_EDGE_DEFAULT) /*!< ADC group regular conversion trigger from external IP: TIM3 TRGO. Trigger edge set to rising edge (default setting). */
#define ADC_REG_TRIG_EXT_TIM15_TRGO     (ADC_CFGR1_EXTSEL_2 | ADC_REG_TRIG_EXT_EDGE_DEFAULT)                      /*!< ADC group regular conversion trigger from external IP: TIM15 TRGO. Trigger edge set to rising edge (default setting). */

// ADC Conversion mode
#define ADC_REG_CONV_SINGLE             (0x00000000U) /*!< ADC conversions are performed in single mode: one conversion per trigger */
#define ADC_REG_CONV_CONTINUOUS         (ADC_CFGR1_CONT)        /*!< ADC conversions are performed in continuous mode: after the first trigger, following conversions launched successively automatically */

// ADC Sequencer discontinuous mode
#define ADC_REG_SEQ_DISCONT_DISABLE     (0x00000000U)                                                          /*!< ADC group regular sequencer discontinuous mode disable */

// ADC DMA
#define ADC_REG_DMA_TRANSFER_NONE       (0x00000000U)              /*!< ADC conversions are not transferred by DMA */
#define ADC_REG_DMA_TRANSFER_LIMITED    (ADC_CFGR1_DMAEN) /*!< ADC conversion data are transferred by DMA, in limited mode (one shot mode): DMA transfer requests are stopped when number of DMA data transfers (number of ADC conversions) is reached. This ADC mode is intended to be used with DMA mode non-circular. */
#define ADC_REG_DMA_TRANSFER_UNLIMITED  (ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN) /*!< ADC conversion data are transferred by DMA, in unlimited mode: DMA transfer requests are unlimited, whatever number of DMA data transferred (number of ADC conversions). This ADC mode is intended to be used with DMA mode circular. */

// ADC Over data behavior
#define ADC_REG_OVR_DATA_PRESERVED      (0x00000000U)/*!< ADC group regular behavior in case of overrun: data preserved */
#define ADC_REG_OVR_DATA_OVERWRITTEN    (ADC_CFGR1_OVRMOD)     /*!< ADC group regular behavior in case of overrun: data overwritten */

// TODO tune timing
#define ADC_DIS_TIMEOUT 5 //ms
// TODO tune timing
#define ADC_CAL_TIMEOUT 5 //ms
// TODO tune timing
#define ADC_ENA_TIMEOUT 5 //ms

// TODO tune timing
#define ADC_STP_TIMEOUT 5 //ms


// this struct is used as a DMA target.
// ADC->DR reads are two bytes, DMA will do half word transfers
// rm0091 tells us the 16->32 bit port mapping packing scheme
// which all us to derive that this struct should be packed
// half words. Additionally, result must be at the end since a
// ref to this struct will be passed into DMAR register for N
// transfers
typedef struct
__attribute__((__packed__)) ADC_Result {
    uint16_t    Motor_current_raw;
    uint16_t    Spin_temperature_raw;
} ADC_Result_t;

void currsen_enable_ht();
void currsen_read_dma();
void currsen_read(ADC_Result_t *res);
CS_Status_t currsen_setup(CS_Mode_t mode, uint8_t motor_adc_ch);
CS_Status_t currsen_adc_cal();
CS_Status_t currsen_adc_en();
CS_Status_t currsen_adc_dis();

float currsen_get_motor_current();
int32_t currsen_get_temperature();
