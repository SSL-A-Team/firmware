/**
 * @file 6step.c
 * @author Will Stuckey
 * @brief core code for 6step commutation of the bldc
 * @version 0.1
 * @date 2022-05-22
 * 
 * @copyright Copyright (c) 2022
 * 
 * Documents you will need:
 *  Understading of 6step PWM commutation:
 *      https://www.youtube.com/user/jtlee1108
 *  Motor Phase/Hall Relationship: 
 *      https://us.nanotec.com/fileadmin/files/Datenblaetter/BLDC/DF45/DF45M024053-A2.pdf
 *  Schematic relating header to the pins:
 *      STEVAL-SPIN3204 Data Brief
 *  Document for STSPIN Listing Specific Peripherial AF Values
 *      stm32f031c6.pdf
 *  Document for stm32f0 class timers
 *      rm0091
 */

#include <stm32f031x6.h>

#include <stdint.h>

/**
 * @brief sets up the pins and timer peripherials associated with the pins 
 * 
 */
void pwm6step_setup() {
    pwm6step_setup_hall_timer();
    pwm6step_setup_commutation_timer();
}

/**
 * @brief sets up the hall sensor timer
 * 
 * The hall sensor timer is TIM2. The hall timer can be used to
 * trigger a TIM1 COM event which will call back it's interrupt handler
 */
void pwm6step_setup_hall_timer() {
    ////////////////
    //  IO setup  //
    ////////////////

    // config as alternate function
    GPIOA->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1);

    // set alternate function to TIM2
    GPIOA->AFR[0] |= (0x2 << GPIO_AFRL_AFSEL0_Pos);
    GPIOA->AFR[0] |= (0x2 << GPIO_AFRL_AFSEL1_Pos);
    GPIOA->AFR[0] |= (0x2 << GPIO_AFRL_AFSEL2_Pos);

    ///////////////////////
    //  TIM2 core setup  //
    ///////////////////////

    // htim2.Init.Prescaler = LF_TIMX_PSC; // 11
    TIM2->PSC = 11;
    // htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM2->CR1 &= ~(TIM_CR1_DIR);
    // htim2.Init.Period = LF_TIMX_ARR; // 24000
    TIM2->ARR = 24000;
    // htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM2->CR1 &= ~(TIM_CR1_CKD_0 | TIM_CR1_CKD_1);
    // htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    TIM2->CR1 &= ~(TIM_CR1_ARPE);

    // enable hall sense interface be selecting the XOR function of input 1-3
    TIM2->CR2 |= TIM_CR2_TI1S;
    // sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    TIM2->SMCR &= ~(TIM_SMCR_SMS_Msk);
    TIM2->SMCR |= (0x4 << TIM_SMCR_SMS_Pos); // b100 reset mode
    //sSlaveConfig.InputTrigger = TIM_TS_TI1F_ED;
    TIM2->SMCR &= ~(TIM_SMCR_TS_Msk);
    TIM2->SMCR |= (0x4 << TIM_SMCR_TS_Pos); // b100 TI1F_ED mode (Timer Input 1 Edge Detect)
    //sSlaveConfig.TriggerFilter = 8;
    TIM2->CCER &= ~(TIM_CCER_CC1E);
    TIM2->CCMR1 &= ~(TIM_CCMR1_IC1F_Msk);
    TIM2->CCMR1 |= (0x8 << TIM_CCMR1_IC1F_Pos);

    //sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
    TIM2->CR2 &= ~TIM_CR2_MMS;
    TIM2->CR2 |= (TIM_CR2_MMS_0 | TIM_CR2_MMS_2); // 0b101 OC2REF for TRGO
    //sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    TIM2->SMCR &= ~TIM_SMCR_MSM;
    TIM2->SMCR |= (TIM_SMCR_MSM);

    ///////////////////////////////////////
    //  TIM2 CH1 setup as delay capture  //
    ///////////////////////////////////////

    /* Disable the Channel 1: Reset the CC1E Bit */
    TIM2->CCER &= ~TIM_CCER_CC1E;

    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM2->CCMR1 |= 0x3 << TIM_CCMR1_CC1S_Pos;

    /* Set the filter */
    TIM2->CCMR1 &= ~TIM_CCMR1_IC1F;
    TIM2->CCMR1 |= ((0x0 << 4U) & TIM_CCMR1_IC1F);

    /* Select the Polarity and set the CC1E Bit */
    TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
    TIM2->CCER |= (0x0U & (TIM_CCER_CC1P | TIM_CCER_CC1NP));

    /* Reset the IC1PSC Bits */
    TIM2->CCMR1 &= ~TIM_CCMR1_IC1PSC;
    //TIM2->CCMR1 |= TIM_CCMR1_IC1PSC;
    TIM2->CCMR1 |= (0 << TIM_CCMR1_IC1PSC_Pos);

    // enable?

    /////////////////////////////
    //  TIM2 CH2 setup as XXX  //
    /////////////////////////////

    /* Disable the Channel 2: Reset the CC2E Bit */
    TIM2->CCER &= ~TIM_CCER_CC2E;

    /* Reset the Output Compare mode and Capture/Compare selection Bits */
    TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
    TIM2->CCMR1 &= ~TIM_CCMR1_CC2S;

    /* Select the Output Compare Mode */
    TIM2->CCMR1 |= (0x7 << TIM_CCMR1_OC2M_Pos);

    /* Set the Output Compare Polarity */
    TIM2->CCER &= ~TIM_CCER_CC2P;
    TIM2->CCER |= (0x0 << TIM_CCER_CC2P_Pos);

    /* Set the Capture Compare Register value */
    TIM2->CCR2 = 0;

    // enable?
}

void pwm6step_setup_commutation_timer() {

}

void TIM2_IRQHandler() {

}

void TIM1_BRK_UP_TRG_COM_IRQHandler() {

}

void pwm6step_set_duty_cycle(int16_t duty_cycle) {

}

void pwm6step_stop() {
    pwm6step_set_duty_cycle(0);
}
