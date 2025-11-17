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
#include "time.h"

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

static bool manual_estop = false;
static bool invert_direction = false;
static bool brake_on_dc_0 = false;

static MotorDirection_t commanded_motor_direction = CLOCKWISE;
static bool direction_change_commanded = false;
static uint16_t current_duty_cycle = 0;
static uint8_t hall_recorded_state_on_transition = 0;
static bool command_brake = false;

/////////////////////
//  hall velocity  //
/////////////////////

static bool hall_speed_estimate_valid = false;
static uint8_t current_hall_value = 0;
static uint8_t prev_hall_value = 0;

//////////////////////
//  error handling  //
//////////////////////

static MotorErrors_t motor_errors;

static int hall_power_error_count = 0;
static int hall_disconnect_error_count = 0;
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

#define COAST_COMMUTATION {false, false, false, false, false, false}

#ifdef BRAKE_HIGH_SIDE
    #define BRAKE_COMMUTATION {false, true,  false, true,  false, true }
#else
    #define BRAKE_COMMUTATION {true,  false, true,  false, true,  false}
#endif

#ifdef BRAKE_ON_HALL_ERROR
    #define HALL_ERROR_COMMUTATION BRAKE_COMMUTATION 
#else
    #define HALL_ERROR_COMMUTATION COAST_COMMUTATION
#endif

static const int ESTOP_COMMUTATION_INDEX = 0;
static const int BRAKE_COMMUTATION_INDEX = 8;
static const int COAST_COMMUTATION_INDEX = 9;

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
 * 1  0  0  1     V  H  G      0   1   0   0   1   0
 * 3  0  1  1     H  V  G      0   0   0   1   1   0
 * 2  0  1  0     G  V  H      1   0   0   1   0   0
 * 6  1  1  0     G  H  V      1   0   0   0   0   1
 * 4  1  0  0     H  G  V      0   0   1   0   0   1
 * 5  1  0  1     V  G  H      0   1   1   0   0   0
 * 
 * Clockwise (direct hall index order)
 * 1  0  0  1     V  H  G      0   1   0   0   1   0
 * 2  0  1  0     G  V  H      1   0   0   1   0   0
 * 3  0  1  1     H  V  G      0   0   0   1   1   0
 * 4  1  0  0     H  G  V      0   0   1   0   0   1
 * 5  1  0  1     V  G  H      0   1   1   0   0   0
 * 6  1  1  0     G  H  V      1   0   0   0   0   1
 * 
 */
static bool cw_commutation_table[10][6] = {
    HALL_ERROR_COMMUTATION,    
    {false, true,  false, false, true,  false},
    {true,  false, false, true,  false, false},
    {false, false, false, true,  true,  false},
    {false, false, true,  false, false, true },
    {false, true,  true,  false, false, false},
    {true,  false, false, false, false, true },
    HALL_ERROR_COMMUTATION,
    BRAKE_COMMUTATION,
    COAST_COMMUTATION
};

static uint8_t cw_expected_hall_transition_table[8] = {
    0x0, // 0 -> 0, error state
    0x5, // 1 -> 5
    0x3, // 2 -> 3
    0x1, // 3 -> 1
    0x6, // 4 -> 6
    0x4, // 5 -> 4
    0x2, // 6 -> 2
    0x7, // 7 -> 7, error state
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
 * 1   0  0  1    V  H  G      1   0   0   0   0   1
 * 5   1  0  1    V  G  H      1   0   0   1   0   0
 * 4   1  0  0    H  G  V      0   0   0   1   1   0
 * 6   1  1  0    G  H  V      0   1   0   0   1   0
 * 2   0  1  0    G  V  H      0   1   1   0   0   0
 * 3   0  1  1    H  V  G      0   0   1   0   0   1
 * 
 * Counter Clockwise (direct hall index order)
 * 1   0  0  1    V  H  G      1   0   0   0   0   1
 * 2   0  1  0    G  V  H      0   1   1   0   0   0
 * 3   0  1  1    H  V  G      0   0   1   0   0   1
 * 4   1  0  0    H  G  V      0   0   0   1   1   0
 * 5   1  0  1    V  G  H      1   0   0   1   0   0
 * 6   1  1  0    G  H  V      0   1   0   0   1   0
 *
 */

static bool ccw_commutation_table[10][6] = {
    HALL_ERROR_COMMUTATION,
    {true,  false, false, false, false, true },
    {false, true,  true,  false, false, false},
    {false, false, true,  false, false, true},
    {false, false, false, true,  true,  false},
    {true,  false, false, true,  false, false},
    {false, true,  false, false, true,  false},
    HALL_ERROR_COMMUTATION,
    BRAKE_COMMUTATION,
    COAST_COMMUTATION
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

//////////////////////////////////////////////
//  internal function forward declarations  //
//////////////////////////////////////////////

static void pwm6step_setup_hall_timer();
static void pwm6step_setup_commutation_timer(uint16_t duty_cycle);
static void TIM2_IRQHandler_HallTransition();
static void perform_commutation_cycle();
static uint8_t read_hall();
static void set_commutation_estop();
static void set_commutation_for_hall(uint8_t hall_value, bool estop);
static void trigger_commutation();
static void load_commutation_values(CommutationValuesType_t);
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
    TIM2->PSC = 11; //div by 11 - 1 = 10, 4.8MHz. Period 208ns 
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
    TIM2->CCMR1 |= (0xF << TIM_CCMR1_IC1F_Pos);

    // set the master mode output trigger to OC2REF
    // TIM2 is a master to TIM1, so an event will trigger COM in TIM1
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

    //  Enable in NVIC

    NVIC_SetPriority(TIM2_IRQn, 5);
    NVIC_EnableIRQ(TIM2_IRQn);

    ////////////////////////////////////////
    //  setup timer for delay triggering  //
    ////////////////////////////////////////

    // there seems to be chatter when TIM1 COM is triggered
    // immediated on a hall transition, so we use TIM16 to
    // create a congiruable delay

    TIM16->SR = 0;
    // prescaler of 48 -> 1MHz input cleck -> each cycle is 1us
    TIM16->PSC = 49;
    TIM16->EGR |= (TIM_EGR_UG);
    // this might should be a function of velocity (estimated or measured)
    TIM16->CCR1 = 61; // commutation delay of 20us

    // enable timer delay for commutation
    TIM16->DIER |= TIM_DIER_CC1IE;
    NVIC_SetPriority(TIM16_IRQn, 5);
    NVIC_EnableIRQ(TIM16_IRQn);
}

#define CCMR1_PHASE1_OFF (TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2)
#define  CCER_PHASE1_OFF (TIM_CCER_CC1E)
#define CCMR1_PHASE1_LOW (TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2)
#define  CCER_PHASE1_LOW (TIM_CCER_CC1NE)
#define CCMR1_PHASE1_HIGH (TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2)
#define  CCER_PHASE1_HIGH (TIM_CCER_CC1E)
#define CCMR1_PHASE1_PWM (TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1CE)
#define  CCER_PHASE1_PWM (TIM_CCER_CC1E | TIM_CCER_CC1NE)
#define CCMR1_PHASE1_PWM_BRAKE (TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1CE)
#define  CCER_PHASE1_PWM_BRAKE (TIM_CCER_CC1NE)

#define CCMR1_PHASE2_OFF (TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2)
#define  CCER_PHASE2_OFF (TIM_CCER_CC2E)
#define CCMR1_PHASE2_LOW (TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_2)
#define  CCER_PHASE2_LOW (TIM_CCER_CC2NE)
#define CCMR1_PHASE2_HIGH (TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_2)
#define  CCER_PHASE2_HIGH (TIM_CCER_CC2E)
#define CCMR1_PHASE2_PWM (TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2CE)
#define  CCER_PHASE2_PWM (TIM_CCER_CC2E | TIM_CCER_CC2NE)
#define CCMR1_PHASE2_PWM_BRAKE (TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2CE)
#define  CCER_PHASE2_PWM_BRAKE (TIM_CCER_CC2NE)

#define CCMR2_PHASE3_OFF (TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_2)
#define  CCER_PHASE3_OFF (TIM_CCER_CC3E)
#define CCMR2_PHASE3_LOW (TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_2)
#define  CCER_PHASE3_LOW (TIM_CCER_CC3NE)
#define CCMR2_PHASE3_HIGH (TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_2)
#define  CCER_PHASE3_HIGH (TIM_CCER_CC3E)
#define CCMR2_PHASE3_PWM (TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3CE)
#define  CCER_PHASE3_PWM (TIM_CCER_CC3E | TIM_CCER_CC3NE)
#define CCMR2_PHASE3_PWM_BRAKE (TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3CE)
#define  CCER_PHASE3_PWM_BRAKE (TIM_CCER_CC3NE)

#define CCMR2_TIM4_ADC_TRIG (TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4CE);
#define CCER_TIM4_ADC_TRIG (TIM_CCER_CC4E)

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

    // set OC_SEL = 0, OC signal visible only to MCU, no action on gate driver
    GPIOA->MODER |= (GPIO_MODER_MODER11_0); // output (def push pull)
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR11_1); // pull down
    GPIOA->BSRR |= GPIO_BSRR_BR_11; // clear bit

    // set OC 100mA, leave standby
    GPIOF->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);
    GPIOF->PUPDR |= (GPIO_PUPDR_PUPDR6_1 | GPIO_PUPDR_PUPDR7_1);
    GPIOF->BSRR |= (GPIO_BSRR_BS_6 | GPIO_BSRR_BR_7);

    // set pin direction to AF
    GPIOA->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);
    GPIOB->MODER |= (GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1);

    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR8_1 | GPIO_PUPDR_PUPDR9_1 | GPIO_PUPDR_PUPDR10_1);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR13_1 | GPIO_PUPDR_PUPDR14_1 | GPIO_PUPDR_PUPDR15_1);

    // set pin alterate function numbers to 0x2 (AF for TIM1)
    GPIOA->AFR[1] |= (0x2 << GPIO_AFRH_AFSEL8_Pos); // PA8
    GPIOA->AFR[1] |= (0x2 << GPIO_AFRH_AFSEL9_Pos); // PA9
    GPIOA->AFR[1] |= (0x2 << GPIO_AFRH_AFSEL10_Pos); // PA10
    GPIOB->AFR[1] |= (0x2 << GPIO_AFRH_AFSEL13_Pos); // PB13
    GPIOB->AFR[1] |= (0x2 << GPIO_AFRH_AFSEL14_Pos); // PB14
    GPIOB->AFR[1] |= (0x2 << GPIO_AFRH_AFSEL15_Pos); // PB15

    ///////////////////////
    //  TIM1 Base Setup  //
    ///////////////////////

    // set the prescaler
    TIM1->PSC = PWM_TIM_PRESCALER;

    // set PWM period relative to the scaled sysclk
    uint16_t arr_pwm_dc_value = (uint16_t) (F_SYS_CLK_HZ / ((uint32_t) pwm_freq_hz * (PWM_TIM_PRESCALER + 1)) - 1);
    TIM1->ARR = arr_pwm_dc_value;

    TIM1->CR1 = (TIM_CR1_ARPE | TIM_CR1_CMS);
    TIM1->CR2 = (TIM_CR2_CCPC);

    TIM1->CCMR1 = CCMR1_PHASE1_OFF | CCMR1_PHASE2_OFF;
    TIM1->CCMR2 = CCMR2_PHASE3_OFF;

    // set channel 4 PWM mode 1, affects OC4REF as input to ADC
    // should be co triggered with ch1-3 commutation
    TIM1->CCMR2 |= CCMR2_TIM4_ADC_TRIG;
    // enable the channel
    TIM1->CCER |= CCER_TIM4_ADC_TRIG;

    // generate an update event to reload the PSC
    TIM1->EGR |= (TIM_EGR_UG | TIM_EGR_COMG);

    TIM1->CR1 |= TIM_CR1_CEN;
    TIM1->BDTR |= TIM_BDTR_MOE;

    // enable interrupt to set GPIOB pin 9 on CH2 PWM low -> high transition
    // used for ADC timing/alignment verification
    // TIM1->SR = 0;
    // TIM1->DIER |= TIM_DIER_CC4IE;
    // NVIC_SetPriority(TIM1_CC_IRQn, 5);
    // NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void TIM2_IRQHandler() {
    // if Capture Compare 1 (hall updated)
    if (TIM2->SR & TIM_SR_CC1IF) {
        // enable timer ch 2 delay
        TIM16->CNT = 0;
        TIM16->CR1 |= (TIM_CR1_CEN);
        TIM16->CCER |= (TIM_CCER_CC1E);

        // read of CCR1 should clear the int enable (PENDING?) flag
        // rm0091, SR, pg 442/1004
        uint32_t hall_transition_elapsed_ticks = TIM2->CCR1;

        // clear interrupt
        TIM2->SR &= ~(TIM_SR_CC1IF);
    }

    // handle errors
}

void TIM16_IRQHandler() {
    if (TIM16->SR & TIM_SR_CC1IF) {
        // disable timer
        TIM16->CCER &= ~(TIM_CCER_CC1E);
        TIM16->CR1 &= ~(TIM_CR1_CEN);
        TIM16->CNT = 0;

        // stage and fire commutation
        TIM2_IRQHandler_HallTransition();

        // clear interrupt
        TIM16->SR &= ~(TIM_SR_CC1IF);
    }
}

void ADC1_IRQHandler() {

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
    // this logic is untested and might be wrong
    // unclear how CC1OF interacts with XOR trigger
    bool had_multiple_transitions = false;
    if (TIM2->SR & TIM_SR_CC1OF) {
        // multiple transitions happened since this interrupt
        // was serviced. Could be noise that escaped the filter
        // or a low mechanical speed (e.g. off) and motor settled
        // on a hall boundary with some env noise
        had_multiple_transitions = true;
    }



    perform_commutation_cycle();
}

/**
 * @brief 
 * 
 */
static void perform_commutation_cycle() {
    // mask off Hall lines PA0-PA2 (already the LSBs)
    // Input Data Register is before all AF muxing so this should be valid always
    //hall_recorded_state_on_transition = (GPIOA->IDR & (GPIO_IDR_2 | GPIO_IDR_1 | GPIO_IDR_0));
    hall_recorded_state_on_transition = read_hall();

    if (hall_recorded_state_on_transition == 0x7) {
        hall_disconnect_error_count += HALL_DISCONNECT_ERROR_INCREMENT;
        if (hall_disconnect_error_count > HALL_DISCONNECT_MAX_ACCU_ERROR) {
            hall_disconnect_error_count = HALL_DISCONNECT_MAX_ACCU_ERROR;
        }
    } else if (hall_disconnect_error_count > 0) {
        hall_disconnect_error_count -= HALL_DISCONNECT_ERROR_CLEAR_DECREMENT;
    }

    if (hall_recorded_state_on_transition == 0x0) {
        hall_power_error_count += HALL_POWER_ERROR_INCREMENT;
        if (hall_power_error_count > HALL_POWER_MAX_ACCU_ERROR) {
            hall_power_error_count = HALL_POWER_MAX_ACCU_ERROR;
        }
    } else if (hall_power_error_count > 0) {
        hall_power_error_count -= HALL_POWER_ERROR_CLEAR_DECREMENT;
    }

    // update error flags
    motor_errors.hall_power = hall_power_error_count > HALL_POWER_ERROR_THRESHOLD;
    motor_errors.hall_disconnected = hall_disconnect_error_count > HALL_DISCONNECT_ERROR_THRESHOLD;

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

    // update prev hall value
    prev_hall_value = hall_recorded_state_on_transition;

    // check for transition error
    // TODO: fix this
    // if (hall_transition_error_count >= HALL_TRANSITION_ERROR_THRESHOLD) {
    //     motor_errors.invalid_transitions = true;
    // }

    // check for errors
    if (motor_errors.hall_power || motor_errors.hall_disconnected) { // || motor_errors.invalid_transitions) {
        // the hardware already performed a COM via TRGO but we'd like to COM the error state
        // manually flag a COM event by setting the bit
        set_commutation_estop();
        trigger_commutation();

        return;
    } 
    
    set_commutation_for_hall(hall_recorded_state_on_transition, false);
    trigger_commutation();
}

/**
 * @brief reads the hall value from the pins
 * 
 * @return uint8_t 
 */
static uint8_t read_hall() {
    uint8_t hall_value = (GPIOA->IDR & (GPIO_IDR_2 | GPIO_IDR_1 | GPIO_IDR_0));

    // for (int i = 0; i < 50; i++) {
    //     uint8_t new_hall_value = (GPIOA->IDR & (GPIO_IDR_2 | GPIO_IDR_1 | GPIO_IDR_0));
    //     if (new_hall_value != hall_value) {
    //         for (;;) {}
    //     }
    // }

    return hall_value;
}

/**
 * @brief stages com values for estop
 * 
 */
static void set_commutation_estop() {
    set_commutation_for_hall(0, true);
}

/**
 * @brief sets channel duty cycles, and stages enables for COM event
 * 
 * @param hall_state 
 */
static void set_commutation_for_hall(uint8_t hall_state, bool estop) {
    bool *commutation_values;
    if (estop) {
        // use whatever error mode was selected for the tables
        commutation_values = cw_commutation_table[ESTOP_COMMUTATION_INDEX];
    } else if (command_brake) {
        commutation_values = cw_commutation_table[BRAKE_COMMUTATION_INDEX];
    } else if (current_duty_cycle == 0) {
        commutation_values = cw_commutation_table[COAST_COMMUTATION_INDEX];
    } else {
        if (commanded_motor_direction == CLOCKWISE) {
            commutation_values = cw_commutation_table[hall_state];
        } else {
            commutation_values = ccw_commutation_table[hall_state];
        }
    }

    bool phase1_low  = commutation_values[0];
    bool phase1_high = commutation_values[1];
    bool phase2_low  = commutation_values[2];
    bool phase2_high = commutation_values[3];
    bool phase3_low  = commutation_values[4];
    bool phase3_high = commutation_values[5];

    // uint16_t arr_pwm_dc_value = (uint16_t) (F_SYS_CLK_HZ / ((uint32_t) PWM_FREQ_HZ * (PWM_TIM_PRESCALER + 1)) - 1);
    // TIM1->CCR4 = arr_pwm_dc_value;

    // uint16_t ccer = 0;
    uint16_t ccer = CCER_TIM4_ADC_TRIG;
    uint16_t ccmr1 = 0;
    // uint16_t ccmr2 = 0;
    uint16_t ccmr2 = CCMR2_TIM4_ADC_TRIG;
    // TIM1->CCR4 = (current_duty_cycle == NUM_RAW_DC_STEPS) ? NUM_RAW_DC_STEPS - MINIMUM_EFFECTIVE_DUTY_CYCLE_RAW : current_duty_cycle;

    TIM1->CCR4 = 22;

    if (phase1_low) {
        if (command_brake) {
            TIM1->CCR1 = current_duty_cycle;
            ccmr1 |= CCMR1_PHASE1_PWM_BRAKE;
            ccer |= CCER_PHASE1_PWM_BRAKE;
        } else {
            TIM1->CCR1 = 0;
            ccmr1 |= CCMR1_PHASE1_LOW;
            ccer |= CCER_PHASE1_LOW;
        }
    } else if (phase1_high) {
        TIM1->CCR1 = current_duty_cycle;
        ccmr1 |= CCMR1_PHASE1_PWM;
        ccer |= CCER_PHASE1_PWM;
    } else {
        TIM1->CCR1 = 0;
        ccmr1 |= CCMR1_PHASE1_OFF;
        ccer |= CCER_PHASE1_OFF;
    }

    if (phase2_low) {
        if (command_brake) {
            TIM1->CCR2 = current_duty_cycle;
            ccmr1 |= CCMR1_PHASE2_PWM_BRAKE;
            ccer |= CCER_PHASE2_PWM_BRAKE;            
        } else {
            TIM1->CCR2 = 0;
            ccmr1 |= CCMR1_PHASE2_LOW;
            ccer |= CCER_PHASE2_LOW;
        }
    } else if (phase2_high) {
        TIM1->CCR2 = current_duty_cycle;
        ccmr1 |= CCMR1_PHASE2_PWM;
        ccer |= CCER_PHASE2_PWM;
    } else {
        TIM1->CCR2 = 0;
        ccmr1 |= CCMR1_PHASE2_OFF;
        ccer |= CCER_PHASE2_OFF;
    }

    if (phase3_low) {
        if (command_brake) {
            TIM1->CCR3 = current_duty_cycle;
            ccmr2 |= CCMR2_PHASE3_PWM_BRAKE;
            ccer |= CCER_PHASE3_PWM_BRAKE;
        } else {
            TIM1->CCR3 = 0;
            ccmr2 |= CCMR2_PHASE3_LOW;
            ccer |= CCER_PHASE3_LOW;
        }
    } else if (phase3_high) {
        TIM1->CCR3 = current_duty_cycle;
        ccmr2 |= CCMR2_PHASE3_PWM;
        ccer |= CCER_PHASE3_PWM;
    } else {
        TIM1->CCR3 = 0;
        ccmr2 |= CCMR2_PHASE3_OFF;
        ccer |= CCER_PHASE3_OFF;
    }

    TIM1->CCMR1 = ccmr1;
    TIM1->CCMR2 = ccmr2;
    TIM1->CCER = ccer;
}

/**
 * @brief trigger a hardware commutation in TIM1
 * 
 */
static void trigger_commutation() {
    TIM1->EGR |= TIM_EGR_COMG;
}

void TIM1_BRK_UP_TRG_COM_IRQHandler() {

}

void TIM1_CC_IRQHandler() {
    GPIOB->BSRR |= GPIO_BSRR_BS_9;
    TIM1->SR &= ~(TIM_SR_CC4IF);
}

/**
 * @brief 
 * 
 * @param duty_cycle 
 * @param motor_direction 
 */
static void pwm6step_set_direct(uint16_t duty_cycle, MotorDirection_t motor_direction) {
    uint16_t scaled_dc = MAP_UINT16_TO_RAW_DC(duty_cycle);
    if (scaled_dc >= MINIMUM_EFFECTIVE_DUTY_CYCLE_RAW 
            && motor_direction != commanded_motor_direction) {
        direction_change_commanded = true;
    }

    // PWM is inverted to facilitate current sensing, so invert user input
    uint32_t scaled_dc_inv = NUM_RAW_DC_STEPS - scaled_dc;

    commanded_motor_direction = motor_direction;
    current_duty_cycle = scaled_dc_inv;

    perform_commutation_cycle();
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
    pwm6step_setup_commutation_timer(PWM_FREQ_HZ);
}

/**
 * @brief 
 * 
 * @param duty_cycle 
 */
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

    command_brake = false;
    pwm6step_set_direct(timer_duty_cycle, motor_direction);
}

void pwm6step_set_duty_cycle_f(float duty_cycle_pct) {
    pwm6step_set_duty_cycle((int32_t) (duty_cycle_pct * (float) UINT16_MAX));
}

void pwm6step_brake(uint16_t braking_force) {
    command_brake = true;
    pwm6step_set_direct(braking_force, commanded_motor_direction);
}

void pwm6step_brake_f(float braking_force_pct) {
    pwm6step_brake((uint16_t) (braking_force_pct * (float) UINT16_MAX));   
}

void pwm6step_stop() {
    pwm6step_set_duty_cycle(0);
}

void pwm6step_estop() {
    pwm6step_set_duty_cycle(0);
    manual_estop = true;
}

void pwm6step_invert_direction(bool invert) {
    invert_direction = invert;
}

bool pwm6step_is_direction_inverted() {
    return invert_direction;
}

const MotorErrors_t pwm6step_get_motor_errors() {
    return motor_errors;
} 

bool pwm6step_hall_rps_estimate_valid() {

}

int pwm6step_hall_get_rps_estimate() {

}


