/**
 * @file main.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stm32f031x6.h>
#include <string.h>

#include "current_sensing.h"
#include "time.h"


////////////////////////////
/// ADC Channel 4 -> PA4 ///
////////////////////////////

static CS_Mode_t cs_mode;

void currsen_enable_ht() {
    ADC1->CR |= ADC_CR_ADSTART;
}

void currsen_read_dma() {
    // GPIOB->BSRR |= GPIO_BSRR_BS_9;
    ADC1->CR |= ADC_CR_ADSTART;
}

void DMA1_Channel1_IRQHandler() {
    // GPIOB->BSRR |= GPIO_BSRR_BR_9;
    DMA1->IFCR |= DMA_IFCR_CGIF1;
}

/**
 * @brief run current sense
 * 
 * @return ADC_Result result of the ADC operation
 */
void currsen_read(ADC_Result_t *res)
{
    const int NUM_CHANNELS = 4;

    ADC1->CR |= ADC_CR_ADSTART;

    //ADC_Result_t res;
    res->status = CS_OK;
    while (1)
    {

        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            // Start ADC conversion
            ADC1->CR |= ADC_CR_ADSTART;

            // Wait until end of conversion
            uint32_t timeout_count = 0;
            while ((ADC1->ISR & ADC_ISR_EOC) == 0)
            {
                if (timeout_count < ADC_DIS_TIMEOUT)
                {
                    //wait_ms(1);
                    //timeout_count += 1;
                }
                else
                {
                    res->status = CS_TIMEOUT;
                    //return res;
                }
            }
            // Store the ADC conversion
            uint16_t currADC = ADC1->DR;
            switch (i)
            {
                case 0:
                    res->I_motor_filt = currADC;
                    //return;
                    break;
                case 1:
                    res->I_motor = currADC;
                    break;
                case 2:
                    res->T_spin = currADC;
                    break;
                case 3:
                    res->V_int = currADC;
                    return;
                    //return res;
            }
        }
    }

}


/**
 * @brief configure, calibrate, and enable the ADC
 * 
 * @return CS_Status_t status of the operation
 */
CS_Status_t currsen_setup(CS_Mode_t mode, ADC_Result_t *res, uint8_t num_ch, uint32_t ch_sel, uint32_t sr_sel)
{
    cs_mode = mode;

    memset(res, 0, sizeof(ADC_Result_t));

    // Assume ADC has not been set up yet
    CS_Status_t status = CS_OK;

    // pre config dma if enabled
    if (cs_mode == CS_MODE_DMA || cs_mode == CS_MODE_TIMER_DMA) {
        DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR));
        DMA1_Channel1->CMAR = (uint32_t) res;
        DMA1_Channel1->CNDTR = num_ch;
        DMA1_Channel1->CCR |= (DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_TEIE | DMA_CCR_CIRC);
        DMA1_Channel1->CCR |= DMA_CCR_EN;

        // DMA1->IFCR |= DMA_IFCR_CGIF1;
        // DMA1_Channel1->CCR |= DMA_CCR_TCIE;

        // NVIC_SetPriority(DMA1_Ch1_IRQn, 5);
        // NVIC_EnableIRQ(DMA1_Ch1_IRQn);
    }

    // Configuration
    status = currsen_adc_conf();
    if (status != CS_OK)
        return status;

    // Group Configuration
    status = currsen_adc_group_config();

    // Calibration
    status = currsen_adc_cal();
    if (status != CS_OK)
        return status;


    // Enable
    status = currsen_adc_en();
    if (status != CS_OK)
        return status;

    // Select ch 3 (pot), ch 4 (motor pin), ch5 (Vbat), ch16 (Temp) ch 17 (Vref), ch 18 (Vbat - internal spec not Vbus motor)
    // in deployed firmware don't check pot, convert motor first
    //ADC1->CHSELR = ADC_CHSELR_CHSEL3 | ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL16 | ADC_CHSELR_CHSEL17; // | ADC_CHSELR_CHSEL18;
    ADC1->CHSELR = ch_sel;
    // ADC1->CHSELR = ADC_CHSELR_CHSEL3;
    // Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than 17.1us
    // at 10B rest 11.5 cycles + (sm 111)=239.5 = 251 cycles
    // useing PCLK / 2 = 48MHz / 2 = 24MHz = 1 / 24_000_000 = 0.000_000_041 ADC clk
    // 251 cycles -> 10.2 us conversion (max precision)
    //
    // PWM @48kHz = period 20.8uS
    // have a max of 5 conversions, < 4us per conversion
    // pick sm 110 = 71.5 clock cycles
    // total conversion = 11.5 + 71.5 = 83 cyc
    // 83 cyc * 0.000_000_041 = 3.4us per sample, 3.4 * 5 = 17us 
    // default 3 cycle + sample cycles * ADC clk 14MHz 0.000_000_071 s per cycle;
    // TODO Need to configure timing based on rise resistance
    //ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2;
    ADC1->SMPR = sr_sel;

    // fires interrupt on end of sequence to drop GPIOB, pin 9
    // used for current sampling/timing verification
    // ADC1->IER |= (ADC_IER_EOSEQIE);
    // ADC1->ISR |= (ADC_ISR_EOSEQ);
    // NVIC_SetPriority(ADC1_IRQn, 6);
    // NVIC_EnableIRQ(ADC1_IRQn);

    return status;
}

void ADC1_IRQHandler() {
    GPIOB->BSRR |= GPIO_BSRR_BR_9;
    ADC1->ISR |= (ADC_ISR_EOSEQ);
}

/**
 * @brief configure the ADC group
 * 
 * @return CS_Status_t status of the operation
 */
CS_Status_t currsen_adc_group_config()
{
    // Software trigger
    // Not discontinuous sampling
    // Continuous conversion
    // No DMA
    // Overwrite data
    MODIFY_REG(ADC1->CFGR1,
                ADC_CFGR1_EXTSEL
               | ADC_CFGR1_EXTEN
               | ADC_CFGR1_DISCEN
               | ADC_CFGR1_CONT
               | ADC_CFGR1_DMAEN
               | ADC_CFGR1_DMACFG
               | ADC_CFGR1_OVRMOD
              ,
                ADC_REG_TRIG_EXT_TIM1_CH4 
                // ((cs_mode == CS_MODE_TIMER_DMA) ? ADC_REG_TRIG_EXT_TIM1_TRGO : ADC_REG_TRIG_SOFTWARE)
               | ADC_REG_SEQ_DISCONT_DISABLE //| ADC_REG_CONV_CONTINUOUS
               | ADC_REG_CONV_SINGLE
               | ADC_REG_DMA_TRANSFER_UNLIMITED // | ADC_REG_DMA_TRANSFER_NONE
               | ADC_REG_OVR_DATA_OVERWRITTEN
              );

    // Wake-up the VREFINT (only for VBAT, Temp sensor and VRefInt) 
    ADC->CCR |= ADC_CCR_VREFEN; // enable internal Vref source
    ADC->CCR |= ADC_CCR_TSEN; // enable internal temp source

    return CS_OK;
}


/**
 * @brief configure the ADC
 * 
 * @return CS_Status_t status of the operation
 */
CS_Status_t currsen_adc_conf()
{
    // Assume ADC not enabled
    // TODO check if ADC enabled

    // Set ADC data resolution
    // 10 bit, 11.5 t_SAR, 1.5 t_SMPL = 13 t_CONV clock cycles
    // Set ADC conversion data alignment - left align
    // Set ADC low power mode - None
    MODIFY_REG(ADC1->CFGR1,
                ADC_CFGR1_RES
               | ADC_CFGR1_ALIGN
               | ADC_CFGR1_WAIT
               | ADC_CFGR1_AUTOFF
              ,
                ADC_RESOLUTION_10B
               | ADC_DATA_ALIGN_LEFT
               | ADC_LP_MODE_NONE
              );

    // Set ADC clock
    // PCLK DIV 2, Latency is deterministic (no jitter) and equal to
    // 2.75 ADC clock cycles
    MODIFY_REG(ADC1->CFGR2,
               ADC_CFGR2_CKMODE
              ,
               ADC_CFGR2_CKMODE_0
              );

    return CS_OK;
}

/**
 * @brief calibrate the ADC
 * 
 * @return CS_Status_t status of the operation
 */
CS_Status_t currsen_adc_cal()
{
    // Ensure that ADEN = 0 before calibration
    if ((ADC1->CR & ADC_CR_ADEN) != 0)
    {
        // Clear ADEN by setting ADDIS to disable
        ADC1->CR |= ADC_CR_ADDIS;
    }

    // Waiting to disable ADC before calibration, timeout if too long
    uint32_t timeout_count = 0;
    while ((ADC1->CR & ADC_CR_ADEN) != 0)
    {
        if (timeout_count < ADC_DIS_TIMEOUT)
        {
            wait_ms(1);
            timeout_count += 1;
        }
        else
        {
            return CS_TIMEOUT;
        }
    }

    // Clear DMAEN
    ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; 
    // Launch the calibration by setting ADCAL 
    ADC1->CR |= ADC_CR_ADCAL; 

    // Waiting for calibration to finish, timeout if too long
    timeout_count = 0;
    while ((ADC1->CR & ADC_CR_ADCAL) != 0)
    {
        if (timeout_count < ADC_CAL_TIMEOUT)
        {
            wait_ms(1);
            timeout_count += 1;
        }
        else
        {
            return CS_TIMEOUT;
        }
    }

    return CS_OK;
}


/**
 * @brief enables the ADC
 * 
 * @return CS_Status_t status of the operation
 */
CS_Status_t currsen_adc_en()
{    
    // Check if ADC is already powered up, and clear if so
    if ((ADC1->ISR & ADC_ISR_ADRDY) != 0)
    {
        // Clear ADRDY 
        ADC1->ISR |= ADC_ISR_ADRDY;
    }

    // Enable the ADC
    ADC1->CR |= ADC_CR_ADEN;
    if (cs_mode == CS_MODE_DMA || cs_mode == CS_MODE_TIMER_DMA) {
        ADC1->CFGR1 |= ADC_CFGR1_DMAEN;
    }
    
    // Waiting for ADC to be powered up, timeout if too long
    uint32_t timeout_count = 0;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0)
    {
        if (timeout_count < ADC_ENA_TIMEOUT)
        {
            wait_ms(1);
            timeout_count += 1;
        }
        else
        {
            return CS_TIMEOUT;
        }
    }

    return CS_OK;
}

/**
 * @brief disables the ADC
 * 
 * @return CS_Status_t status of the operation
 */
CS_Status_t currsen_adc_dis()
{ 
    // Stop any ongoing conversion 
    ADC1->CR |= ADC_CR_ADSTP;

    // Waiting until conversion is stopped
    uint32_t timeout_count = 0;
    while ((ADC1->CR & ADC_CR_ADSTP) != 0)
    {
        if (timeout_count < ADC_STP_TIMEOUT)
        {
            wait_ms(1);
            timeout_count += 1;
        }
        else
        {
            return CS_TIMEOUT;
        }
    }

    // Disable the ADC
    ADC1->CR |= ADC_CR_ADDIS;
    // Waiting until ADC is fully disabled
    timeout_count = 0;
    while ((ADC1->CR & ADC_CR_ADEN) != 0)
    {
        if (timeout_count < ADC_DIS_TIMEOUT)
        {
            wait_ms(1);
            timeout_count += 1;
        }
        else
        {
            return CS_TIMEOUT;
        }
    }

    return CS_OK;
}