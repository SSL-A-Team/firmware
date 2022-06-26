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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stm32f031x6.h>

#include "6step.h"
#include "system.h"

typedef enum MotorDirection {
    CLOCKWISE,
    COUNTER_CLOCKWISE
} MotorDirection_t;

typedef enum CommutationValuesType {
    NORMAL,
    MOMENTUM_DETECTED,
    ERROR_DETECTED
} CommutationValuesType_t;

//////////////////////
//  motor commands  //
//////////////////////

static bool invert_direction = false;

static MotorDirection_t commanded_motor_direction = CLOCKWISE;
static bool direction_change_commanded = false;
static uint16_t current_duty_cycle = 0;

static uint8_t hall_recorded_state_on_transition = 0;

/////////////////////
//  hall velocity  //
/////////////////////

static bool hall_speed_estimate_valid = false;
static uint8_t current_hall_value = 0;
static uint8_t prev_hall_value = 0;

//////////////////////
//  error handling  //
//////////////////////

static bool has_error_latched = false;

static bool has_hall_power_error = false;
static bool has_hall_disconnect_error = false;
static int hall_power_error_count = 0;
static int hall_disconnect_error_count = 0;

static bool has_hall_transition_error = false;
static int hall_transition_error_count = 0;

////////////////////////////////
//  local data and functions  //
////////////////////////////////

#define CH1_FULL (TIM_CCER_CC1E | TIM_CCER_CC1NE)
#define CH2_FULL (TIM_CCER_CC2E | TIM_CCER_CC2NE)
#define CH3_FULL (TIM_CCER_CC3E | TIM_CCER_CC3NE)

#define CH1_AND_CH2 (CH1_FULL | CH2_FULL)
#define CH2_AND_CH3 (CH2_FULL | CH3_FULL)
#define CH1_AND_CH3 (CH1_FULL | CH3_FULL)

#define COAST_CHANNEL_EN_MAP  (0)
#define COAST_CHANNEL_DIS_MAP (CH1_FULL | CH2_FULL | CH3_FULL)
#define BREAK_CHANNEL_EN_MAP  (TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE)
#define BREAK_CHANNEL_DIS_MAP (TIM_CCER_CC1E  | TIM_CCER_CC2E  | TIM_CCER_CC3E )

#ifdef BREAK_ON_HALL_ERROR
    #define HALL_ERROR_CH_EN_MAP   BREAK_CHANNEL_EN_MAP
    #define HALL_ERROR_CH_DIS_MAP  BREAK_CHANNEL_DIS_MAP
    #define HALL_ERROR_COMMUTATION {true,  false, true,  false, true,  false}  
#else
    #define HALL_ERROR_CH_EN_MAP   COAST_CHANNEL_EN_MAP
    #define HALL_ERROR_CH_DIS_MAP  COAST_CHANNEL_DIS_MAP
    #define HALL_ERROR_COMMUTATION {false, false, false, false, false, false}
#endif

/**
 * @brief clockwise transition table
 * 
 * Hall sensors should produce a gray-coded 3-bit value, meaning
 * 0 (0'b000) and 7 (0'b111) are invalid. The signal wires have
 * pull up resistors so 7 probably means one or more wires is unplugged,
 * and 0 probably means there's a power issue.
 * 
 * Clockwise (human readable order)
 * D  H3 H2 H1 -> P1 P2 P3 -> W1L W1H W2L W2H W3L W3H
 * 1   0  0  1    H  G  V      0   0   1   0   0   1
 * 5   1  0  1    V  G  H      0   1   1   0   0   0
 * 4   1  0  0    V  H  G      0   1   0   0   1   0
 * 6   1  1  0    H  V  G      0   0   0   1   1   0
 * 2   0  1  0    G  V  H      1   0   0   1   0   0
 * 3   0  1  1    G  H  V      1   0   0   0   0   1
 * 
 * Clockwise (direct hall index order)
 * 1   0  0  1    H  G  V      0   0   1   0   0   1
 * 2   0  1  0    G  V  H      1   0   0   1   0   0
 * 3   0  1  1    G  H  V      1   0   0   0   0   1
 * 4   1  0  0    V  H  G      0   1   0   0   1   0
 * 5   1  0  1    V  G  H      0   1   1   0   0   0
 * 6   1  1  0    H  V  G      0   0   0   1   1   0
 *
 */
static bool cw_commutation_table[8][6] = {
    HALL_ERROR_COMMUTATION,
    {false, false, true,  false, false, true },
    {true,  false, false, true,  false, false},
    {true,  false, false, false, false, true },
    {false, true,  false, false, true,  false},
    {false, true,  true,  false, false, false},
    {false, false, false, true,  true,  false},
    HALL_ERROR_COMMUTATION
};

static uint8_t cw_expected_hall_transition_table[8] = {
    0x0, // 0 -> 0, error state
    0x5, // 1 -> 5
    0x3, // 2 -> 3
    0x1, // 3 -> 1
    0x6, // 4 -> 6
    0x4, // 5 -> 4
    0x2, // 6 -> 2
    0x7, // 7 -> 4, error state
};

static uint32_t cw_commutation_ch_enable_map[8] = {
    HALL_ERROR_CH_EN_MAP,
    CH2_AND_CH3,
    CH1_AND_CH2,
    CH1_AND_CH3,
    CH1_AND_CH3,
    CH1_AND_CH2,
    CH2_AND_CH3,
    HALL_ERROR_CH_EN_MAP,
};

static uint32_t cw_commutation_ch_disable_map[8] = {
    HALL_ERROR_CH_DIS_MAP,
    CH1_FULL,
    CH3_FULL,
    CH2_FULL,
    CH2_FULL,
    CH3_FULL,
    CH1_FULL,
    HALL_ERROR_CH_DIS_MAP,
};


/**
 * @brief counter clockwise transition table
 * 
 * Hall sensors should produce a gray-coded 3-bit value, meaning
 * 0 (0'b000) and 7 (0'b111) are invalid. The signal wires have
 * pull up resistors so 7 probably means one or more wires is unplugged,
 * and 0 probably means there's a power issue.
 * 
 * Counter Clockwise (human readable order)
 * D  H3 H2 H1 -> P1 P2 P3 -> W1L W1H W2L W2H W3L W3H 
 * 1  0  0  1     V  H  G      0   1   0   0   1   0
 * 3  0  1  1     H  V  G      0   0   0   1   1   0
 * 2  0  1  0     G  V  H      1   0   0   1   0   0
 * 6  1  1  0     G  H  V      1   0   0   0   0   1
 * 4  1  0  0     H  G  V      0   0   1   0   0   1
 * 5  1  0  1     V  G  H      0   1   1   0   0   0
 * 
 * Counter Clockwise (direct hall index order)
 * 1  0  0  1     V  H  G      0   1   0   0   1   0
 * 2  0  1  0     G  V  H      1   0   0   1   0   0
 * 3  0  1  1     H  V  G      0   0   0   1   1   0
 * 4  1  0  0     H  G  V      0   0   1   0   0   1
 * 5  1  0  1     V  G  H      0   1   1   0   0   0
 * 6  1  1  0     G  H  V      1   0   0   0   0   1
 * 
 */
static bool ccw_commutation_table[8][6] = {
    HALL_ERROR_COMMUTATION,    
    {false, true,  false, false, true,  false},
    {true,  false, false, true,  false, false},
    {false, false, false, true,  true,  false},
    {false, false, true,  false, false, true },
    {false, true,  true,  false, false, false},
    {true,  false, false, false, false, true },
    HALL_ERROR_COMMUTATION
};

static uint8_t ccw_expected_hall_transition_table[8] = {
    0x0, // 0 -> 0, error state
    0x3, // 1 -> 3
    0x6, // 2 -> 6
    0x2, // 3 -> 2
    0x5, // 4 -> 5
    0x1, // 5 -> 1
    0x4, // 6 -> 4
    0x7, // 7 -> 7, error state
};

static uint32_t ccw_commutation_ch_enable_map[8] = {
    HALL_ERROR_CH_EN_MAP,
    CH1_AND_CH3,
    CH1_AND_CH2,
    CH2_AND_CH3,
    CH2_AND_CH3,
    CH1_AND_CH2,
    CH1_AND_CH3,
    HALL_ERROR_CH_EN_MAP,
};

static uint32_t ccw_commutation_ch_disable_map[8] = {
    HALL_ERROR_CH_DIS_MAP,
    CH2_FULL,
    CH3_FULL,
    CH1_FULL,
    CH1_FULL,
    CH3_FULL,
    CH2_FULL,
    HALL_ERROR_CH_DIS_MAP,
};

//////////////////////////////////////////////
//  internal function forward declarations  //
//////////////////////////////////////////////

static void pwm6step_setup_hall_timer();
static void pwm6step_setup_commutation_timer(uint16_t duty_cycle);
//TIM2_IRQHandler();
static void TIM2_IRQHandler_HallTransition();
static void TIM2_IRQHandler_TIM1CommutationComplete();
static void load_commutation_values(CommutationValuesType_t);
//TM1_BRK_UP_TRG_COM_IRQHandler();
static void pwm6step_set_direct(uint16_t, MotorDirection_t);

/**
 * @brief sets up the hall sensor timer
 * 
 * The hall sensor timer is TIM2. The hall timer can be used to
 * trigger a TIM1 COM event which will call back it's interrupt handler
 */
static void pwm6step_setup_hall_timer() {
    //////////////////////////
    //  IO setup            //
    //   - PA0 <- HALL1/A+  //
    //   - PA1 <- HALL2/B+  //
    //   - PA2 <- HALL3/C+  //
    //////////////////////////

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
    TIM2->ARR = 24000; // probs wrong, but unused rn
    // htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM2->CR1 &= ~(TIM_CR1_CKD_0 | TIM_CR1_CKD_1);
    // htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    TIM2->CR1 &= ~(TIM_CR1_ARPE);

    // enable hall sense interface be selecting the XOR function of input 1-3
    TIM2->CR2 |= TIM_CR2_TI1S;
    // sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    // clear SMS (required to set TS)
    TIM2->SMCR &= ~(TIM_SMCR_SMS_Msk);
    //sSlaveConfig.InputTrigger = TIM_TS_TI1F_ED;
    TIM2->SMCR &= ~(TIM_SMCR_TS_Msk);
    TIM2->SMCR |= (0x4 << TIM_SMCR_TS_Pos); // b100 TI1F_ED mode (Timer Input 1 Edge Detect)
    // now that TS is set, set SMS
    TIM2->SMCR |= (0x4 << TIM_SMCR_SMS_Pos); // b100 reset mode
    //sSlaveConfig.TriggerFilter = 8;
    TIM2->CCER &= ~(TIM_CCER_CC1E);
    TIM2->CCMR1 &= ~(TIM_CCMR1_IC1F_Msk);
    TIM2->CCMR1 |= (0x8 << TIM_CCMR1_IC1F_Pos);

    // set the master mode output trigger to OC2REF
    // TIM2 is a master to TIM1, so an event will trigget COM in TIM1
    // the source of outbound trigger will be Output Compare Channel 2 REF (count down to 0)
    // this allows us to delay COM until the hall edge detection interrupt finishes
    TIM2->CR2 &= ~TIM_CR2_MMS;
    TIM2->CR2 |= (TIM_CR2_MMS_0 | TIM_CR2_MMS_2); // 0b101 OC2REF for TRGO
    // enable master slave mode 
    TIM2->SMCR &= ~TIM_SMCR_MSM;
    TIM2->SMCR |= (TIM_SMCR_MSM);

    ///////////////////////////////////////
    //  TIM2 CH1 setup as delay capture  //
    ///////////////////////////////////////

    /* Disable the Channel 1: Reset the CC1E Bit */
    TIM2->CCER &= ~TIM_CCER_CC1E;

    // only writeable when channel is off
    // CC1S = 2'b11 -> sets input capture on TRC (internal trigger set by TS bits)
    // this is done in general config above, TRC source is edge detect
    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM2->CCMR1 |= (0x3 << TIM_CCMR1_CC1S_Pos);

    // disable filter, not used when edge detect TRC is selected
    TIM2->CCMR1 &= ~TIM_CCMR1_IC1F;
    TIM2->CCMR1 |= ((0x0 << 4U) & TIM_CCMR1_IC1F);

    // set edge detection mode to non-inverted/rising edge.
    // it is forbidden to use dual edge detect for hall sensing mode
    TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
    TIM2->CCER |= (0x0U & (TIM_CCER_CC1P | TIM_CCER_CC1NP));

    // set the channel prescaler to 0
    TIM2->CCMR1 &= ~TIM_CCMR1_IC1PSC;
    TIM2->CCMR1 |= (0 << TIM_CCMR1_IC1PSC_Pos);

    // enable the interrupt
    TIM2->DIER |= (TIM_DIER_CC1IE);
    TIM2->SR &= ~(TIM_SR_CC1IF);

    // enable channel 1
    TIM2->CCER |= TIM_CCER_CC1E;

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

    //////////////////////
    //  Enable in NVIC  //
    //////////////////////

    NVIC_SetPriority(TIM2_IRQn, 5);
    NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief 
 * 
 * @param pwm_freq_hz 
 */
static void pwm6step_setup_commutation_timer(uint16_t pwm_freq_hz) {
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

    // set pin speed to high frequency
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

    // break on failure
    TIM1->BDTR |= TIM_BDTR_BKE;

    // set dead time
    TIM1->BDTR |= DEAD_TIME;
}

void TIM2_IRQHandler() {
    // while (true) {
    //     GPIOB->BSRR |= GPIO_BSRR_BS_9;
    //     wait_ms(1000);
    //     GPIOB->BSRR |= GPIO_BSRR_BR_9;
    //     wait_ms(1000);
    // }

    // if Capture Compare 1 (hall updated)
    if (TIM2->SR & TIM_SR_CC1IF) {
        TIM2_IRQHandler_HallTransition();
    }

    // if CC2 (output timer which delays COM event, call *after* COM)
    if (TIM2->SR & TIM_SR_CC2IF) {
        TIM2_IRQHandler_TIM1CommutationComplete();
    }

    // handle errors
}

/**
 * @brief 
 * 
 * keep this interrupt short so we can minimize the delay between the CC1
 * transition event and delayed required on CC2 to fire the COM event automatically
 * 
 * handle most functionality on the interrupt callback for CC2 which fires *after*
 * the COM event
 * 
 * force apply O0 as we'll need to count the instructions here + ctxswitch to make
 * sure this function completes before the other interrupt fires. Maybe that happens
 * anyway since read of CCR1 clear the IF
 * 
 */
__attribute__((optimize("O0")))
static void TIM2_IRQHandler_HallTransition() {
    bool had_multiple_transitions = false;
    if (TIM2->SR & TIM_SR_CC1OF) {
        // multiple transitions happened since this interrupt
        // was serviced. Could be noise that escaped the filter
        // or a low mechanical speed (e.g. off) and motor settled
        // on a hall boundary with some env noise
        had_multiple_transitions = true;
    }

    // mask off Hall lines PA0-PA2 (already the LSBs)
    // Input Data Register is before all AF muxing so this should be valid always
    hall_recorded_state_on_transition = (GPIOA->IDR & (GPIO_IDR_2 | GPIO_IDR_1 | GPIO_IDR_0));

    // read of CCR1 should clear the int enable flag
    // rm0091, SR, pg 442/1004
    uint32_t hall_transition_elapsed_ticks = TIM2->CCR1;
}

static void TIM2_IRQHandler_TIM1CommutationComplete() {
    if (hall_recorded_state_on_transition == 0) {
        hall_power_error_count++;
    }

    if (hall_recorded_state_on_transition == 0x7) {
        hall_disconnect_error_count++;
    }

    if (hall_power_error_count > HALL_POWER_ERROR_THRESHOLD) {
        has_hall_power_error = true;
    }

    if (hall_disconnect_error_count > HALL_DISCONNECT_ERROR_THRESHOLD) {
        has_hall_disconnect_error = true;
    }

    uint8_t expected_transition = 0;
    if (commanded_motor_direction == CLOCKWISE) {
        expected_transition = cw_expected_hall_transition_table[prev_hall_value];  // TODO bad state
    } else {
        expected_transition = ccw_expected_hall_transition_table[prev_hall_value];
    }

    bool carry_forward_momentum_detected = false;
    if (hall_recorded_state_on_transition != expected_transition) {
        // the mechanical system has momentum
        // just because the system was commanded to change
        // direction doesn't mean the next transition will
        // be in the correct direction. We'll accept transitions
        // as long as they are valid in any direction, until
        // a transition is recorded in the correct direction
        if (direction_change_commanded) {
            uint8_t reverse_direction_transition = 0x0;
            if (commanded_motor_direction == CLOCKWISE) {
                reverse_direction_transition = ccw_expected_hall_transition_table[prev_hall_value];
            } else {
                reverse_direction_transition = cw_expected_hall_transition_table[prev_hall_value];
            }

            if (hall_recorded_state_on_transition == reverse_direction_transition) {
                carry_forward_momentum_detected = true;
            } else {
                hall_transition_error_count++;
            }
        } else {
            hall_transition_error_count++;
        }
    } else {
        // we got the expected transition in the commanded direction so
        // we should clear the direction_change_commanded flag
        if (direction_change_commanded) {
            direction_change_commanded = false;
        }
    }

    prev_hall_value = hall_recorded_state_on_transition;

    if (hall_transition_error_count >= HALL_TRANSITION_ERROR_THRESHOLD) {
        has_hall_transition_error = true;
    }

    if (has_hall_power_error || has_hall_disconnect_error || has_hall_transition_error) {
        has_error_latched = true;

        // the hardware already performed a COM via TRGO but we'd like to COM the error state
        // manually flag a COM event by setting the bit

        load_commutation_values(ERROR_DETECTED);

        TIM1->EGR |= TIM_EGR_COMG;

        return;
    }

    if (carry_forward_momentum_detected) {
        // the hardware already performed a COM via TRGO
        // but the motor continued spinning in the un-commanded direction
        // this means the commutation is off by one step, which
        // produces a weaker reverse effect
        //
        // there's probably a nasty edge case here if we 
        //
        // restage commutation and manually flag a COM

        load_commutation_values(MOMENTUM_DETECTED);

        TIM1->EGR |= TIM_EGR_COMG;

        // still load next values under the assumption that direction will change and
        // we can fix it again next step if necessary
    }

    load_commutation_values(NORMAL);

    // clear interrupt pending bit of Capture Compare CH2
    TIM2->SR &= ~(TIM_SR_CC2IF);
}

/**
 * @brief 
 * 
 * @param commutation_type 
 */
static void load_commutation_values(CommutationValuesType_t commutation_type) {
    bool *commutation_values;
    uint32_t enable_mask;
    uint32_t disable_mask;
    if (commutation_type == ERROR_DETECTED) {
        // use whatever error mode was selected for the tables
        commutation_values = cw_commutation_table[0x0];
        enable_mask = cw_commutation_ch_enable_map[0x0];
        disable_mask = ccw_commutation_ch_disable_map[0x0];
    } else if (commutation_type == MOMENTUM_DETECTED) {
        if (commanded_motor_direction == CLOCKWISE) {
            commutation_values = cw_commutation_table[hall_recorded_state_on_transition];
            enable_mask = cw_commutation_ch_enable_map[hall_recorded_state_on_transition];
            disable_mask = cw_commutation_ch_disable_map[hall_recorded_state_on_transition];
        } else {
            commutation_values = ccw_commutation_table[hall_recorded_state_on_transition];
            enable_mask = ccw_commutation_ch_enable_map[hall_recorded_state_on_transition];
            disable_mask = ccw_commutation_ch_disable_map[hall_recorded_state_on_transition];
        }
    } else {
        // normal commutation
        uint8_t expected_next_hall_step = 0x0;
        if (commanded_motor_direction == CLOCKWISE) {
            expected_next_hall_step = cw_expected_hall_transition_table[hall_recorded_state_on_transition];
        } else {
            expected_next_hall_step = ccw_expected_hall_transition_table[hall_recorded_state_on_transition];
        }

        if (commanded_motor_direction == CLOCKWISE) {
            commutation_values = cw_commutation_table[expected_next_hall_step];
            enable_mask = cw_commutation_ch_enable_map[expected_next_hall_step];
            disable_mask = cw_commutation_ch_disable_map[expected_next_hall_step];
        } else {
            commutation_values = ccw_commutation_table[expected_next_hall_step];
            enable_mask = ccw_commutation_ch_enable_map[expected_next_hall_step];
            disable_mask = ccw_commutation_ch_disable_map[expected_next_hall_step];
        }
    }

    bool phase1_high = commutation_values[0];
    bool phase1_low  = commutation_values[1];
    bool phase2_high = commutation_values[2];
    bool phase2_low  = commutation_values[3];
    bool phase3_high = commutation_values[4];
    bool phase3_low  = commutation_values[5];

    if (phase1_low) {
        TIM1->CCR1 = 0;
    } else if (phase1_high) {
        TIM1->CCR1 = current_duty_cycle;
    }

    if (phase2_low) {
        TIM1->CCR2 = 0;
    } else if (phase2_high) {
        TIM2->CCR2 = current_duty_cycle;
    }

    if (phase3_low) {
        TIM1->CCR3 = 0;
    } else if (phase3_high) {
        TIM2->CCR3 = current_duty_cycle;
    }

    TIM1->CCER &= ~(disable_mask);
    TIM1->CCER |= (enable_mask);
}

/**
 * @brief 
 * 
 */
static void TIM1_BRK_UP_TRG_COM_IRQHandler() {
    // don't think this is actually used
    // hardware does all transitions on the COM event
    // still need to clear IT pending bit
}

/**
 * @brief 
 * 
 * @param duty_cycle 
 * @param motor_direction 
 */
static void pwm6step_set_direct(uint16_t duty_cycle, MotorDirection_t motor_direction) {
    uint16_t old_dc = current_duty_cycle;

    commanded_motor_direction = motor_direction;
    current_duty_cycle = duty_cycle;

    // motor was stopped or commanded to stop
    // jump start
    if (old_dc == 0) {
        hall_recorded_state_on_transition = (GPIOA->IDR & (GPIO_IDR_2 | GPIO_IDR_1 | GPIO_IDR_0));

        load_commutation_values(MOMENTUM_DETECTED);
        TIM1->EGR |= TIM_EGR_COMG;
        load_commutation_values(NORMAL);
    }
}

////////////////////////
//  Public Functions  //
////////////////////////

/**
 * @brief sets up the pins and timer peripherials associated with the pins 
 * 
 */
void pwm6step_setup() {
    pwm6step_setup_hall_timer();
    //pwm6step_setup_commutation_timer(PWM_FREQ_HZ);
}

void pwm6step_set_duty_cycle(int32_t duty_cycle) {
    MotorDirection_t motor_direction;
    if (duty_cycle < 0) {
        motor_direction = COUNTER_CLOCKWISE;
    } else {
        motor_direction = CLOCKWISE;
    }

    if (invert_direction) {
        if (motor_direction == COUNTER_CLOCKWISE) {
            motor_direction = CLOCKWISE;
        } else {
            motor_direction = COUNTER_CLOCKWISE;
        }
    }

    uint32_t duty_cycle_abs = abs(duty_cycle);
    if (duty_cycle_abs > UINT16_MAX) {
        duty_cycle_abs = UINT16_MAX;
    }

    uint16_t timer_duty_cycle = (uint16_t) duty_cycle_abs;

    pwm6step_set_direct(timer_duty_cycle, motor_direction);
}

void pwm6step_stop() {
    pwm6step_set_duty_cycle(0);
}

void pwm6step_invert_direction(bool invert) {
    invert_direction = invert;
}

bool pwm6step_is_direction_inverted() {
    return invert_direction;
}

bool pwm6step_hall_rps_estimate_valid() {

}

int pwm6step_hall_get_rps_estimate() {

}


