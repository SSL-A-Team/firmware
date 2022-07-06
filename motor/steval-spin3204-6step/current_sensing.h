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

struct ADC_Result_t {
    CS_Status_t status;
    uint16_t    motor0;
    uint16_t    vref;
    uint16_t    vbatt;
};

ADC_Result_t currsen_read();
CS_Status_t currsen_setup();

