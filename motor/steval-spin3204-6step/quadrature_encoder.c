/**
 * @brief 
 * 
 * @param timer 
 */

#include <stm32f031x6.h>

void quadenc_setup();

////////////////////////////////////////////
//  private function forward definitions  //
////////////////////////////////////////////

void quadenc_setup() {
    // Set PA6 & PA7 to AF mode
    GPIOA->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
    GPIOA->AFR[0] |= (0x1U << GPIO_AFRL_AFSEL6_Pos);
    GPIOA->AFR[0] |= (0x1U << GPIO_AFRL_AFSEL7_Pos);
    
    TIM3->CCMR1 |= TIM_CCMR1_CC1S_0; 
    TIM3->CCMR2 |= TIM_CCMR1_CC2S_0;
    TIM3->SMCR |= (0x3U << TIM_SMCR_SMS_Pos);
    TIM3->CCER |= (0x1U << TIM_CCER_CC1P_Pos);
    TIM3->CCER |= (0x1U << TIM_CCER_CC2P_Pos);
    TIM3->CCER &= ~(1U << TIM_CCER_CC1NP_Pos);
    TIM3->CCER &= ~(1U << TIM_CCER_CC2NP_Pos);
    TIM3->ARR |= (0x8000U << TIM_ARR_ARR_Pos);
    TIM3->CR1 |= (TIM_CR1_ARPE);
    TIM3->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);
    TIM3->CR1 |= (0x1U << TIM_CR1_CEN_Pos);
    TIM3->EGR |= TIM_EGR_UG;
}

uint32_t quadenc_get_counter() {
    return TIM3->CCR1 + TIM3->CCR2;
}