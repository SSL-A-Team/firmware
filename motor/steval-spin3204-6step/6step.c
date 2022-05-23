/**
 * @file 6step.c
 * @author Will Stuckey
 * @brief core code for 6step commutation of the bldc
 * @version 0.1
 * @date 2022-05-22
 * 
 * @copyright Copyright (c) 2022
 * 
 * The most readable file ever! Jk. Its not too bad but you'll definitely
 * need stm32f031c6 and rm0091 in hand (or memorized LOL) before continuing.
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

#include <stdint.h>
#include <stm32f031x6.h>

#include "6step.h"
#include "system.h"


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

/**
 * @brief 
 * 
 * @param pwm_freq_hz 
 */
void pwm6step_setup_commutation_timer(uint16_t pwm_freq_hz) {
    ////////////////////////////
    //  TIM1 Setup IO         //
    //      - PA8  TIM1_CH1   //
    //      - PA9  TIM1_CH2   //
    //      - PA10 TIM1_CH3   //
    //      - PB13 TIM1_CH1N  //
    //      - PB14 TIM1_CH2N  //
    //      - PB15 TIM1-CH3N  //
    ////////////////////////////

    // IO bank clocks initialized in setup.c

    // set pin direction to AF
    GPIOA->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);
    GPIOB->MODER |= (GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1);

    // set pin alterate function numbers to 0x2 (AF for TIM1)
    GPIOA->AFR[1] |= (0x2 << GPIO_AFRH_AFRH1_Pos); // PA8
    GPIOA->AFR[1] |= (0x2 << GPIO_AFRH_AFRH2_Pos); // PA9
    GPIOA->AFR[1] |= (0x2 << GPIO_AFRH_AFRH3_Pos); // PA10
    GPIOB->AFR[1] |= (0x2 << GPIO_AFRH_AFRH5_Pos); // PB13
    GPIOB->AFR[1] |= (0x2 << GPIO_AFRH_AFRH6_Pos); // PB14
    GPIOB->AFR[1] |= (0x2 << GPIO_AFRH_AFRH7_Pos); // PB15

    // set pin spin to high frequency
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR9_0 | GPIO_OSPEEDR_OSPEEDR9_1);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR10_0 | GPIO_OSPEEDR_OSPEEDR10_1);    
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR13_0 | GPIO_OSPEEDR_OSPEEDR13_1);
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR14_0 | GPIO_OSPEEDR_OSPEEDR14_1);
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR15_0 | GPIO_OSPEEDR_OSPEEDR15_1);

    ///////////////////////
    //  TIM1 Base Setup  //
    ///////////////////////

    // set counter mode up (clear DIR bit)
    TIM1->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);

    // set clock divider to 1
    TIM1->CR1 &= ~(TIM_CR1_CKD);

    // disable auto load/reload
    TIM1->CR1 &= ~(TIM_CR1_ARPE_Msk);

    // set the prescaler
    TIM1->PSC = PWM_TIM_PRESCALER;

    // set PWM period relative to the scaled sysclk
    TIM1->ARR = (uint16_t) (F_SYS_CLK_HZ / ((uint32_t) pwm_freq_hz * (PWM_TIM_PRESCALER + 1)) - 1);

    // clear/disable rep counter
    TIM1->RCR = 0;

    // generate an update event to reload the PSC
    TIM1->EGR = TIM_EGR_UG;

    ///////////////////////////////
    //  TIM1 Clock Source Setup  //
    ///////////////////////////////

    // sync clear of SMS, TS, ETF, ETPS, ECE, ETP bits
    TIM1->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_TS | TIM_SMCR_ETF | TIM_SMCR_ETPS | TIM_SMCR_ECE | TIM_SMCR_ETP);

    //////////////////////
    //  TIM1 PWM Setup  // 
    //////////////////////
    // gotta love that ST HAL, seemingly identical to TIM1 Base Setup

    ////////////////////////////////
    //  TIM1 OC Ref Setup Ch 1-3  //
    ////////////////////////////////

    TIM1->SMCR &= ~(TIM_SMCR_OCCS);

    // set polarity
    TIM1->SMCR &= ~(TIM_SMCR_ETP_Msk);
    // set prescaler
    TIM1->SMCR &= ~(TIM_SMCR_ETPS_Msk);
    // set filter
    TIM1->SMCR |= (0x8 << TIM_SMCR_ETF_Pos);

    TIM1->SMCR |= TIM_SMCR_OCCS;

    // CH1
    TIM1->CCMR1 |= TIM_CCMR1_OC1CE;
    // CH2
    TIM1->CCMR1 |= TIM_CCMR1_OC2CE;
    // CH3
    TIM1->CCMR1 |= TIM_CCMR2_OC3CE;

    //////////////////////////////////
    //  TIM1 Master + Slave Config  //
    //////////////////////////////////

    // set trigger source ITR1 (TIM2 hall sensors)
    TIM1->SMCR &= ~(TIM_SMCR_TS_Msk);
    TIM1->SMCR |= (0x1 << TIM_SMCR_TS_Pos);

    // set slave mode to reset
    TIM1->SMCR &= ~(TIM_SMCR_SMS_Msk);
    TIM1->SMCR |= (0x4 << TIM_SMCR_SMS_Pos); // slave mode reset

    // set TRGO
    TIM1->CR2 &= ~(TIM_CR2_MMS_Msk);
    TIM1->CR2 |= (0x7 << TIM_CR2_MMS_Pos); // master output trigger is OC4REF 

    // enable master slave sync
    TIM1->SMCR &= ~(TIM_SMCR_MSM);
    TIM1->SMCR |= TIM_SMCR_MSM;

    //////////////////////////////////
    //  TIM1 PWM Channel 1-4 Setup  //
    //////////////////////////////////


    // CH1
    // disable the channel
    TIM1->CCER &= ~(TIM_CCER_CC1E);

    // set mode to PWM1
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S);
    TIM1->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);

    // set polarity LOW
    TIM1->CCER &= ~(TIM_CCER_CC1P);
    TIM1->CCER |= TIM_CCER_CC1P;

    // set complementary channel polarity LOW
    TIM1->CCER &= ~(TIM_CCER_CC1NP);
    TIM1->CCER |= TIM_CCER_CC1NP;

    // reset complementary state
    TIM1->CCER &= ~(TIM_CCER_CC1NE);

    // reset/clear OC state RESET
    TIM1->CR2 &= ~(TIM_CR2_OIS1 | TIM_CR2_OIS1N);

    // set the duty cycle for channel 1
    TIM1->CCR1 = 0;

    // enable?
    //

    // set the preload enable bit
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;

    // clear/diable fast mode
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1FE);


    // CH2
    // disable the channel
    TIM1->CCER &= ~(TIM_CCER_CC2E);

    // set mode to PWM1
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_CC2S);
    TIM1->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);

    // set polarity LOW
    TIM1->CCER &= ~(TIM_CCER_CC2P);
    TIM1->CCER |= TIM_CCER_CC2P;

    // set complementary channel polarity LOW
    TIM1->CCER &= ~(TIM_CCER_CC2NP);
    TIM1->CCER |= TIM_CCER_CC2NP;

    // reset complementary state
    TIM1->CCER &= ~(TIM_CCER_CC2NE);

    // reset/clear OC state RESET
    TIM1->CR2 &= ~(TIM_CR2_OIS2 | TIM_CR2_OIS2N);

    // set the duty cycle for channel 2
    TIM1->CCR2 = 0;

    // enable?
    //

    // set the preload enable bit
    TIM1->CCMR1 |= TIM_CCMR1_OC2PE;

    // clear/diable fast mode
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC2FE);
    

    // CH3
    // disable the channel
    TIM1->CCER &= ~(TIM_CCER_CC3E);

    // set mode to PWM1
    TIM1->CCMR1 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_CC3S);
    TIM1->CCMR1 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);

    // set polarity LOW
    TIM1->CCER &= ~(TIM_CCER_CC3P);
    TIM1->CCER |= TIM_CCER_CC3P;

    // set complementary channel polarity LOW
    TIM1->CCER &= ~(TIM_CCER_CC3NP);
    TIM1->CCER |= TIM_CCER_CC3NP;

    // reset complementary state
    TIM1->CCER &= ~(TIM_CCER_CC3NE);

    // reset/clear OC state RESET
    TIM1->CR2 &= ~(TIM_CR2_OIS3 | TIM_CR2_OIS3N);

    // set the duty cycle for channel 3
    TIM1->CCR3 = 0;

    // enable?
    //

    // set the preload enable bit
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;

    // clear/diable fast mode
    TIM1->CCMR2 &= ~(TIM_CCMR2_OC3FE);


    // CH4
    // disable the channel
    TIM1->CCER &= ~(TIM_CCER_CC4E);

    // set mode to PWM1
    TIM1->CCMR1 &= ~(TIM_CCMR2_OC4M | TIM_CCMR2_CC4S);
    TIM1->CCMR1 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2);

    // set polarity HIGH
    TIM1->CCER &= ~(TIM_CCER_CC4P);

    // reset/clear OC state RESET
    TIM1->CR2 &= ~(TIM_CR2_OIS4);

    // set the duty cycle for channel 4
    TIM1->CCR4 = 0;

    // enable?
    //

    // set the preload enable bit
    TIM1->CCMR2 |= TIM_CCMR2_OC4PE;

    // clear/diable fast mode
    TIM1->CCMR2 &= ~(TIM_CCMR2_OC4FE);
    

    //////////////////////////////
    //  TIM1 break + dead time  //
    //////////////////////////////


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
