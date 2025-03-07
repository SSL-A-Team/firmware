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

static CS_Mode_t m_cs_mode;
static ADC_Result_t m_adc_result;

void currsen_enable_ht() {
    ADC1->CR |= ADC_CR_ADSTART;
}

void currsen_read_dma() {
    ADC1->CR |= ADC_CR_ADSTART;
}

void DMA1_Channel1_IRQHandler() {
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
CS_Status_t currsen_setup(CS_Mode_t mode, uint8_t motor_adc_ch)
{
    m_cs_mode = mode;

    memset(m_adc_result, 0, sizeof(ADC_Result_t));

    // Assume ADC has not been set up yet
    CS_Status_t status = CS_OK;

    // pre config dma if enabled
    if (m_cs_mode == CS_MODE_DMA || m_cs_mode == CS_MODE_TIMER_DMA) {
        // Disable DMA1 Channel 1 before configuring.
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        // Set DMA to 16-bit memory size (0b01)
        DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;
        DMA1_Channel1->CCR &= ~DMA_CCR_MSIZE_1;
        // Set DMA to 16-bit peripheral size (0b01)
        DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;
        DMA1_Channel1->CCR &= ~DMA_CCR_PSIZE_1;
        // Set DMA to circular mode
        DMA1_Channel1->CCR |= DMA_CCR_CIRC;
        // Set DMA Channel 1 transfer error interrupt enable.
        DMA1_Channel1->CCR |= DMA_CCR_TEIE;
        // Set DMA Channel 1 half transfer interrupt and transfer complete interrupt to disable.
        DMA1_Channel1->CCR &= ~(DMA_CCR_HTIE | DMA_CCR_TCIE);
        // Set DMA Channel 1 direction to read from peripheral.
        DMA1_Channel1->CCR &= ~DMA_CCR_DIR;
        // Set DMA Channel 1 priority to very high.
        DMA1_Channel1->CCR |= DMA_CCR_PL;

        // Set DMA Channel 1 Peripheral Address to ADC1 Data Register.
        DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR));
        // Set DMA Channel 1 Memory Address to the result struct.
        DMA1_Channel1->CMAR = (uint32_t) res;
        // Set DMA Channel 1 Number of Data to Transfer to the number of channels.
        // Temperature and motor so 2.
        // Since in circular mode, this will reset.
        DMA1_Channel1->CNDTR = 2;
        // Enable DMA1 Channel 1.
        // TODO: Need to find DMA Enable in ST source.
        DMA1_Channel1->CCR |= DMA_CCR_EN;
    }

    // Disable ADC before configuration.
    status = currsen_adc_dis();
    if (status != CS_OK)
        return status;

    // TODO Add ADC watchdog?

    // Set ADC data resolution
    // Based on Table 46 of RM0091.
    // 12 bit has a t_SAR = 12.5 ADC clock cycles (have margin to do so below).
    // Set ADC conversion data alignment - left align
    MODIFY_REG(
        ADC1->CFGR1,
        ADC_CFGR1_RES |
        ADC_CFGR1_ALIGN,
        ADC_RESOLUTION_12B |
        ADC_DATA_ALIGN_LEFT);

    // Set ADC low power mode - None
    MODIFY_REG(
        ADC1->CFGR1,
        (ADC_CFGR1_WAIT | ADC_CFGR1_AUTOFF),
        ADC_LP_MODE_NONE);

    // Set ADC clock
    // Based on Table 44 of RM0091.
    // PCLK DIV 4, Latency on starting sample is deterministic.
    // t_LATENCY = 2.625 ADC clock cycles.
    // PCLK (48 MHz) / 4 = 12 MHz ADC clock.
    // Must be DIV 4 for 48 MHz PCLK to be less than 14 MHz ADC clock.
    // NOTE: ST does clock async CKMODE = 00. We are not doing the
    // fastest (14 MHz > 12 MHz) but there is no jitter on start timing.
    // From Table 50 of stm32f031c4.pdf
    // W_LATENCY = 8.5 ADC clock cycles for PCLK/4.
    MODIFY_REG(
        ADC1->CFGR2,
        ADC_CFGR2_CKMODE,
        ADC_CFGR2_CKMODE_1);

    // TIM1 Ch4 Falling Edge trigger
    // TIM1 -> Triggering 48 kHz
    // 1 / 48 kHz = 20.833 us period
    // Falling edge is used because the fall of the PWM is closer to center
    // than the rising edge. Probably doesn't actually matter because
    // we aren't making 48 kHz control adjustments (today).
    // Not discontinuous sampling.
    // Single conversion.
    // Unlimited DMA transfers.
    // Overwrite data on conversion.
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
        | ADC_REG_SEQ_DISCONT_DISABLE
        | ADC_REG_CONV_SINGLE
        | ADC_REG_DMA_TRANSFER_UNLIMITED
        | ADC_REG_OVR_DATA_OVERWRITTEN);

    // Temperature sensor values
    // Table 53. STM32F031x6.pdf
    // ts_temp minimum ADC sampling time -> 4 us
    // At 12 MHz ADC clock -> 48 ADC clock cycles.
    // SMP must be 101 (55.5 ADC clock cycles) to 111 (239.5) for temperature sensor.
    // Wake-up the Temperature Sensor
    ADC->CCR |= (ADC_CCR_TSEN);

    // Since sampling + conversion period must be less than
    // 20.833 us, we need to set the sampling time to be less, so
    // SMP must be less than 111 (239.5 ADC clock cycles).

    // Total sampling + conversion time
    // From Figure 31 of RM0091
    // Total first conversion -> (t_LATENCY + t_SMPL + t_SAR + W_LATENCY)
    // All following conversions -> (t_SMPL + t_SAR + W_LATENCY)
    // t_SMPL -> based on SMP[2:0] bits. Min is 1.5 and max is 239.5 ADC clock cycles (13.11.6 in RM0091).
    // Will go with SMP = 110 (71.5 ADC clock cycles) for now as motor should be low pass filtered.
    // Total first conversion -> (2.625 + 71.5 + 12.5 + 8.5) = 95.125 ADC clock cycles.
    // first conversion * (1 / 12 MHz ADC Clock) = 7.92708 us for 1 channel.
    // All following conversions -> (71.5 + 12.5 + 8.5) = 92.5 ADC clock cycles.
    // following conversions * (1 / 12 MHz ADC Clock) = 7.7083 us for each additional.
    // So if we do one motor and temp sensor, we are at 7.92708 + 7.7083 = 15.63538 us.

    ADC1->SMPR = ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1;

    // Set ADC channel selection
    // Channel 16 is the temperature sensor.
    ADC1->CHSELR = motor_adc_ch | ADC_CHSELR_CHSEL16;

    // Calibration
    status = currsen_adc_cal();
    if (status != CS_OK)
        return status;

    // Enable
    status = currsen_adc_en();
    if (status != CS_OK)
        return status;

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
 * @brief disables the ADC. Procedure pulled directly from RM0091 page 238.
 *
 * @return CS_Status_t status of the operation
 */
CS_Status_t currsen_adc_dis()
{
    // Check if ADC is already disabled
    // (not enabled and not converting).
    if ((ADC1->CR & ADC_CR_ADEN) == 0 &&
        (ADC1->CR & ADC_CR_ADSTART) == 0)
    {
        return CS_OK;
    }

    // Stop any ongoing conversion.
    ADC1->CR |= ADC_CR_ADSTP;

    // Waiting until conversion is stopped.
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
    // Waiting until ADC is fully disabled.
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

/**
 * @brief gets the motor current. Translate from raw ADC value to
 *       to scaled raw ADC value to motor current.
 *
 * @return float motor current
 */
float currsen_get_motor_current()
{
    // TODO need more for motor current scaling
    return V_ADC_SCALE * m_adc_result.Motor_current_raw;
}

/**
 * @brief gets the temperature. Translate from raw ADC value to
 *       to scaled raw ADC value to temperature.
 *
 * @return int32_t temperature in C
 */

int32_t currsen_get_temp()
{
    // From A.7.16 of RM0091
    int32_t temperature; /* will contain the temperature in degrees Celsius */
    // Scale the raw ADC value to the VDD_CALIB value and center based on the
    // temperature calibration points.
    temperature = (((int32_t) m_adc_result.Spin_temperature_raw * VDD_APPLI / VDD_CALIB) - (int32_t) *TEMP30_CAL_ADDR);
    // Scale by the difference between the two calibration points.
    temperature = temperature * (int32_t)(110 - 30);
    temperature = temperature / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
    // Add the offset of 30 degrees Celsius from the base calibration point.
    temperature = temperature + 30;
    return temperature;
}
