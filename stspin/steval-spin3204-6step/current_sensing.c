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
#include <time.h>
#include <current_sensing.h>


////////////////////////////
/// ADC Channel 4 -> PA4 ///
////////////////////////////

/**
 * @brief run current sense
 * 
 * @return ADC_Result result of the ADC operation
 */
ADC_Result_t currsen_read()
{
    // Select ch 4 (motor pin), ch 17 (Vref), ch 18 (Vbat) 
    ADC1->CHSELR = ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL17 | ADC_CHSELR_CHSEL18;
    // Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than 17.1us
    // TODO Need to configure timing based on rise resistance
    ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2;

    // Wake-up the VREFINT (only for VBAT, Temp sensor and VRefInt) 
    ADC->CCR |= ADC_CCR_VREFEN;
    ADC_Result_t res;
    res.status = CS_OK;
    while (1)
    {
        // Start ADC conversion
        ADC1->CR |= ADC_CR_ADSTART;
        for (i=0; i < 3; i++)
        {
            // Wait until end of conversion
            uint32_t timeout_count = 0;
            while ((ADC1->ISR & ADC_ISR_EOC) == 0)
            {
                if (timeout_count < ADC_DIS_TIMEOUT)
                {
                    wait_ms(1);
                    timeout_count += 1;
                }
                else
                {
                    res.status = CS_TIMEOUT;
                    return res;
                }
            }
            // Store the ADC conversion
            uint16_t currADC = ADC1->DR;
            switch (i)
            {
                case 0:
                    res.motor0 = currADC;
                    break;
                case 1:
                    res.vref = currADC;
                    break;
                case 2:
                    res.vbatt = currADC;
                    return res;
            }
        }
    }

}


/**
 * @brief configure, calibrate, and enable the ADC
 * 
 * @return CS_Status_t status of the operation
 */
CS_Status_t currsen_setup()
{
    // Assume ADC has not been set up yet
    CS_Status_t status = CS_OK;

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

    return status;
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
                ADC_REG_TRIG_SOFTWARE
               | ADC_REG_SEQ_DISCONT_DISABLE
               | ADC_REG_CONV_CONTINUOUS
               | ADC_REG_DMA_TRANSFER_NONE
               | ADC_REG_OVR_DATA_OVERWRITTEN
              );

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
    // 10 bit, 11.5 t_SAR, 1.5 t_SMPL, 13 t_CONV clock cycles
    // Set ADC conversion data alignment - right align
    // Set ADC low power mode - None
    MODIFY_REG(ADCx->CFGR1,
                ADC_CFGR1_RES
               | ADC_CFGR1_ALIGN
               | ADC_CFGR1_WAIT
               | ADC_CFGR1_AUTOFF
              ,
                ADC_RESOLUTION_10B
               | ADC_DATA_ALIGN_RIGHT
               | ADC_LP_MODE_NONE
              );

    // Set ADC clock
    // PCLK DIV 2, Latency is deterministic (no jitter) and equal to
    // 2.75 ADC clock cycles
    MODIFY_REG(ADCx->CFGR2,
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
    timeout = 0;
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

    return CS_OK:
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
        ADC1->ISR |= ADC_CR_ADRDY;
    }

    // Enable the ADC
    ADC1->CR |= ADC_CR_ADEN;
    
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