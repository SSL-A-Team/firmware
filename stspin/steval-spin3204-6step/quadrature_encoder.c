/**
 * @file quadrature_encoder.c
 * @author Will Stuckey & Austin Jones
 * @brief functions for the STSPIN32 encoder interface counter
 * @version 0.1
 * @date 2022-06-12
 * 
 * @copyright Copyright (c) 2022
 * 
 * Relevant Documents:
 * stm32f0x1/stm32f0x2/stm32f0x8 reference "m0 reference"
 *      pp. 422-424
 * 
 */

#include <stm32f031x6.h>

/////////////////////////
//  private variables  //
/////////////////////////

static uint16_t prev_enc_value = 0;

///////////////////////////////////
//  public function definitions  //
///////////////////////////////////

/**
 * @brief setup the encoder in pins PA6 and PA7
 * 
 */
void quadenc_setup() {
    ///////////////////
    // ENC_A -> PA6  //
    // ENC_B -> PA7  //
    ///////////////////

    // Set PA6 & PA7 to AF mode
    GPIOA->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
    // set speed rating 10MHz
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR6_0 | GPIO_OSPEEDR_OSPEEDR7_0);
    // select AF1, TIM3_CH1 and TIM3_CH2
    GPIOA->AFR[0] |= (0x1U << GPIO_AFRL_AFSEL6_Pos);
    GPIOA->AFR[0] |= (0x1U << GPIO_AFRL_AFSEL7_Pos);

    // set CCMR = 0'b01, TI1FP1 -> TI1, TI2FP2 -> TI2
    TIM3->CCMR1 |= TIM_CCMR1_CC1S_0; 
    TIM3->CCMR2 |= TIM_CCMR1_CC2S_0;
    // instructed to clear/keep these clear...
    TIM3->CCER &= ~(1U << TIM_CCER_CC1NP_Pos);
    TIM3->CCER &= ~(1U << TIM_CCER_CC2NP_Pos);
    // both Trigger Inputs 1 & 2 sensitive on both rising and falling edges
    TIM3->SMCR |= (0x3U << TIM_SMCR_SMS_Pos);
    // flip the polarity of both channels (we think we need to do this based on the scope waveform)
    // maybe we don't need this
    //TIM3->CCER |= (0x1U << TIM_CCER_CC1P_Pos);
    //TIM3->CCER |= (0x1U << TIM_CCER_CC2P_Pos);
    TIM3->CR1 |= (TIM_CR1_CEN);
}

/**
 * @brief gets the raw counter value
 * 
 * @return uint16_t 
 */
uint16_t quadenc_get_counter() {
    // counter clockwise counts down
    //  - underflow resets to 65535
    // clockwise counts up
    //  - overflow to 0

    return TIM3->CNT;
}

/**
 * @brief resets the counter to the median value
 * 
 * NOTE: make sure its infeasible for 32k tick to occur
 *       in either direction between calls to get the
 *       delta
 * 
 */
void quadenc_reset_encoder_delta() {
    TIM3->CNT = 0x8000;
}

/**
 * @brief get change in encoder ticks over time (positive is CW, negative is CCW)
 * 
 * NOTE: make sure its infeasible for 32k tick to occur
 *       in either direction between calls to get the
 *       delta
 * TODO: investigate OF/UF detection so we know if the delta is bad
 * 
 * @return int32_t number of encoder ticks since the last call
 */
int32_t quadenc_get_encoder_delta() {
    uint16_t cur_val = quadenc_get_counter();
    quadenc_reset_encoder_delta();

    int32_t enc_delta = (int32_t) cur_val - 0x8000;
    return enc_delta;
}