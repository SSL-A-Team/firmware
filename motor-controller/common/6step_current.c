#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stm32f031x6.h>

#include "6step_current.h"
#include "current_sensing.h"
#include "pid.h"
#include "system.h"
#include "time.h"

typedef enum MotorDirection {
    CLOCKWISE,
    COUNTER_CLOCKWISE
} MotorDirection_t;

//////////////////////
//  motor commands  //
//////////////////////

typedef enum _MotorControlMode {
    DUTY = 0,
    VOLTAGE  = 1,
    CURRENT = 2,
} MotorControlMode;

static volatile MotorControlMode motor_control_mode = DUTY;
static volatile MotorDirection_t commanded_motor_direction = CLOCKWISE;
static volatile uint16_t current_duty_cycle = 0;
static volatile uint8_t hall_recorded_state_on_transition = 0;

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

///////////////////
//  Current PID  //
///////////////////

const Uint16FixedPoint_t S0F16_4096_OVER_9000 = 29826;

static FixedPointS12F4_PiConstants_t current_controller_constants = {
    // 1000 Hz bandwidth -> 6283 rads

    // .kP = 2123,
    // .kI = 910,

    // KNOWN GOOD
    .kP = 338 * 3,      // S07F10, 6283 * 0.00033 H = 2.07339 => 2123
    .kI = 145 * 3,  // S05F13, 6283 * (0.7ohm coil + 0.007 ohm wire) * (1 / 40000) = 0.11105 => 910 
    .kI_max = 4095,  // S12F0
    .kI_min = -(4095),  // S12F0
    .anti_jitter_thresh = 0,
    .anti_jitter_thresh_inv = 0,
    // .anti_jitter_thresh = 30,  // S12F0
    // .anti_jitter_thresh_inv = 135,  // S0F12
};

static FixedPointS12F4_PiController_t current_controller;

static volatile uint16_t measured_current = 0;
static volatile uint16_t measured_vbus_voltage = 0;
static volatile uint16_t last_voltage_command_mv = 0;

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
static void pwm6step_setup_commutation_timer();
static void TIM2_IRQHandler_HallTransition();
static void perform_commutation_cycle();
static uint8_t read_hall();
static void set_commutation_estop();
static void set_commutation_for_hall(uint8_t hall_value, bool estop);
static void trigger_commutation();

static void set_duty_cycle(uint16_t duty_cycle);
static void set_voltage(uint16_t voltage_mv);
static void set_current(uint16_t current_ma);


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

	TIM2->CR1 = 0;
	// XOR TI1 to TI3, reset signal used as TRGO
	TIM2->CR2 = TIM_CR2_TI1S;
	// TI1 (filtered) edge detector as TRGI, slave mode: reset
	TIM2->SMCR = TIM_SMCR_TS_2 | TIM_SMCR_SMS_2;
	TIM2->SR = 0;
	TIM2->DIER = 0;
	// IC1 is mapped to TI1 filtered (XOR output)
	TIM2->CCMR1 = TIM_CCMR1_CC1S | (7 << TIM_CCMR1_IC1F_Pos);
	TIM2->CCMR2 = 0;
	// CCR1 in input mode, capture all edges of TI1F
	TIM2->CCER = TIM_CCER_CC1E;
	TIM2->CNT = 0;
	TIM2->PSC = 0; // => 48MHz
	TIM2->ARR = UINT32_MAX;
	TIM2->EGR = TIM_EGR_UG;

    NVIC_SetPriority(TIM2_IRQn, 5);
    NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 |= TIM_CR1_CEN;
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
static void pwm6step_setup_commutation_timer() {
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
    TIM1->ARR = NUM_RAW_DC_STEPS - 1;

    TIM1->CR1 = (TIM_CR1_ARPE | TIM_CR1_CMS);
    TIM1->CR2 = (TIM_CR2_CCPC);
	TIM1->SMCR = TIM_SMCR_OCCS | TIM_SMCR_ETF | TIM_SMCR_TS_0;

    // set channel 4 PWM mode 1, affects OC4REF as input to ADC
    // should be co triggered with ch1-3 commutation
    TIM1->CCMR1 = CCMR1_PHASE1_OFF | CCMR1_PHASE2_OFF;
    TIM1->CCMR2 = CCMR2_PHASE3_OFF | CCMR2_TIM4_ADC_TRIG;
    // enable the channel
    TIM1->CCER |= CCER_PHASE1_OFF | CCER_PHASE2_OFF | CCER_PHASE3_OFF | CCER_TIM4_ADC_TRIG;
    // set adc trigger offset
    TIM1->CCR4 = 22;

    // generate an update event to reload the PSC
    TIM1->EGR |= (TIM_EGR_UG | TIM_EGR_COMG);

    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;

    perform_commutation_cycle();
    trigger_commutation();
}

void TIM2_IRQHandler()
{
	// hall commutation IRQ
	TIM2->SR &= ~TIM_SR_UIF;

    uint32_t hall_transition_elapsed_ticks = TIM2->CCR1;

    perform_commutation_cycle();
}

static uint32_t voltage_filter[4];
static size_t voltage_filter_index = 0;

static uint32_t current_filter[32];
static size_t current_filter_ind = 0;

static volatile size_t double_buffer_ind = 0;
static uint16_t current_data_buffer[2][20];
static uint32_t logging_2frame_avg_cur = 0;

static uint16_t adc_callback_ctr = 0;

static volatile bool flag_1ms = false;

/**
 * should be called at 40Khz
 */
// void ADC1_IRQHandler() {
void DMA1_Channel1_IRQHandler() {
    // clear DMA interrupt flags
    DMA1->IFCR = DMA_IFCR_CGIF1;

    // clear ADC converstion flags
	ADC1->ISR = ADC_ISR_EOSMP | ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_OVR;

    // read current
    Uint32FixedPoint_t current_measurement = currsen_get_shunt_current_ma();  // U15
    // we need to normalize some values so the fixed point arith has reasonable intermediate computation sizes (32bit)
    // quantize current_measurement to 4096 (12 bits)
    // Max current the sense network should read is ~9A -> 9000mA
    if (current_measurement > 9000) {
        current_measurement = 9000;
    }

    // this is a 2.2kHz bandwidth filter with sampling freq of 40kHz

    // filter current
    current_filter[current_filter_ind++] = current_measurement;
    // current_filter_ind &= 0x7;
    // current_filter_ind &= 0xF;
    current_filter_ind &= 0x1F;


    Uint32FixedPoint_t avg_current = 0;
    for (int i = 0; i < sizeof(current_filter) / 4; i++) {
        avg_current += current_filter[i];
    }
    avg_current >>= 5;  // div 16

    measured_current = avg_current;

    // update bus voltage

    uint16_t vbus_mv = currsen_get_vbus_voltage_mv();
    // measured_vbus_voltage = vbus_mv;

    voltage_filter[voltage_filter_index++] = vbus_mv;
    voltage_filter_index &= 0x3;

    uint32_t average_bus_voltage = 0;
    for (int i = 0; i < sizeof(voltage_filter) / 4; i++) {
        average_bus_voltage += voltage_filter[i];
    }
    average_bus_voltage >>= 2;

    measured_vbus_voltage = average_bus_voltage;

    // quantize average current to 0-4096 for PID loop
    // S15F0 * S0F16 = S15F16 >> 19 = S12F0
    Uint32FixedPoint_t qtz_current = (avg_current * S0F16_4096_OVER_9000) >> 16;

    // update PID, maps current to voltage
    fxptpi_calculate(&current_controller, qtz_current, 0);
    Int32FixedPoint_t voltage_sp = fxptpi_get_output(&current_controller);
    if (voltage_sp < 0) {
        voltage_sp = 0;
    }

    // TODO scale voltage_sp back up to 25200mV
    // note: set_voltage will need to compute a scale from measured voltage
    // as a the actual max the PWM is commanding, to the DC
    // as we've observed, the RPM/power output is quite different from
    // max battery to minimum and certainly if the voltage further dips under load
    // we want to correct for this 

    // scale from 4096 -> battery voltage
    uint16_t voltage_sp_mv = ((uint32_t) voltage_sp * BATTERY_VOLTAGE_MV) >> 12;

    // if we're not in CURRENT mode, then the user has directly set the voltage/dc via the public function
    // in that case, this callback is collecting logging data and averaging/update bus voltage for VOLTAGE mode
    if (motor_control_mode == CURRENT) {
        set_voltage(voltage_sp_mv);    
    }

    if ((adc_callback_ctr & 0x1) == 0) {
        logging_2frame_avg_cur = avg_current;
    } else {
        current_data_buffer[double_buffer_ind][adc_callback_ctr / 2] = (uint16_t) ((logging_2frame_avg_cur + (uint32_t) avg_current) >> 1);
    }

    adc_callback_ctr++;
    if (adc_callback_ctr == PWM_FREQ_HZ / 1000) {
        flag_1ms = true;

        adc_callback_ctr = 0;
        double_buffer_ind = (double_buffer_ind + 1) & 0x1;
    }

    trigger_commutation();
}

/**
 * @brief 
 * 
 */
static void perform_commutation_cycle() {
    hall_recorded_state_on_transition = read_hall();

    uint8_t cw_expected_transition = cw_expected_hall_transition_table[prev_hall_value];
    uint8_t ccw_expected_transition = ccw_expected_hall_transition_table[prev_hall_value];


    if (hall_recorded_state_on_transition != prev_hall_value 
            && hall_recorded_state_on_transition != cw_expected_transition 
            && hall_recorded_state_on_transition != ccw_expected_transition) {
        hall_transition_error_count++;
    } else if (motor_errors.invalid_transitions > 0) {
        hall_transition_error_count--;
    }

    // latch transition error
    if (hall_transition_error_count >= HALL_TRANSITION_ERROR_THRESHOLD) {
        motor_errors.invalid_transitions = true;
    }

    // update prev hall value
    prev_hall_value = hall_recorded_state_on_transition;

    // check for errors
    if (motor_errors.hall_power || motor_errors.hall_disconnected) { // || motor_errors.invalid_transitions) {
        set_commutation_estop();
        return;
    } 
    
    set_commutation_for_hall(hall_recorded_state_on_transition, false);
}

/**
 * @brief reads the hall value from the pins
 * 
 * @return uint8_t 
 */
static uint8_t read_hall() {
    uint8_t hall_value = (GPIOA->IDR & (GPIO_IDR_2 | GPIO_IDR_1 | GPIO_IDR_0));
    
    // check if hall sensor is floating/disconnected, all lines will be 1, 3b'111 = 7
    if (hall_value == 0x7) {
        hall_disconnect_error_count += HALL_DISCONNECT_ERROR_INCREMENT;
        if (hall_disconnect_error_count > HALL_DISCONNECT_MAX_ACCU_ERROR) {
            hall_disconnect_error_count = HALL_DISCONNECT_MAX_ACCU_ERROR;
        }
    } else if (hall_disconnect_error_count > 0) {
        hall_disconnect_error_count -= HALL_DISCONNECT_ERROR_CLEAR_DECREMENT;
    }

    // check if hall sensor has a power error, pullups will fail all lines will be 0, 3b'000 = 7
    if (hall_value == 0x0) {
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


    // uint16_t ccer = 0;
    uint16_t ccer = CCER_TIM4_ADC_TRIG;
    uint16_t ccmr1 = 0;
    uint16_t ccmr2 = CCMR2_TIM4_ADC_TRIG;

    if (phase1_low) {
        ccmr1 |= CCMR1_PHASE1_LOW;
        ccer |= CCER_PHASE1_LOW;
    } else if (phase1_high) {
        ccmr1 |= CCMR1_PHASE1_PWM;
        ccer |= CCER_PHASE1_PWM;
    } else {
        ccmr1 |= CCMR1_PHASE1_OFF;
        ccer |= CCER_PHASE1_OFF;
    }

    if (phase2_low) {
        ccmr1 |= CCMR1_PHASE2_LOW;
        ccer |= CCER_PHASE2_LOW;
    } else if (phase2_high) {
        ccmr1 |= CCMR1_PHASE2_PWM;
        ccer |= CCER_PHASE2_PWM;
    } else {
        ccmr1 |= CCMR1_PHASE2_OFF;
        ccer |= CCER_PHASE2_OFF;
    }

    if (phase3_low) {
        ccmr2 |= CCMR2_PHASE3_LOW;
        ccer |= CCER_PHASE3_LOW;
    } else if (phase3_high) {
        ccmr2 |= CCMR2_PHASE3_PWM;
        ccer |= CCER_PHASE3_PWM;
    } else {
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
    void;
}

void TIM1_CC_IRQHandler() {
    GPIOB->BSRR |= GPIO_BSRR_BS_9;
    TIM1->SR &= ~(TIM_SR_CC4IF);
}

////////////////////////
//  Public Functions  //
////////////////////////

/**
 * @brief sets up the pins and timer peripherials associated with the pins 
 * 
 */
void pwm6step_setup() {
    // setup current PI controller
    fxptpi_initialize(&current_controller, &current_controller_constants);

    pwm6step_setup_commutation_timer();
    pwm6step_setup_hall_timer();
}

/**
 * 
 */
void pwm6step_set_direction(MotorDirection_t motor_direction) {
    if (INVERT_MOTOR_DIRECTION) {
        if (motor_direction == COUNTER_CLOCKWISE) {
            motor_direction = CLOCKWISE;
        } else {
            motor_direction = COUNTER_CLOCKWISE;
        }
    }

    commanded_motor_direction = motor_direction;
}

static void set_duty_cycle(uint16_t duty_cycle) {
    current_duty_cycle = ARR_VALUE - MAP_MAX_DUTY_TO_ARR_DUTY(duty_cycle);

    // set drive registers
    TIM1->CCR1 = current_duty_cycle;
    TIM1->CCR2 = current_duty_cycle;
    TIM1->CCR3 = current_duty_cycle;

    if (TIM2->DIER == 0) {
        // enable it and force an update
        TIM2->DIER = TIM_DIER_UIE;
        TIM2->EGR = TIM_EGR_UG;
    }
}

/**
 * @brief 
 * 
 * @param duty_cycle_arr 
 */
void pwm6step_set_duty_cycle(int16_t duty_cycle) {
    if (duty_cycle < 0) {
        pwm6step_set_direction(COUNTER_CLOCKWISE);
    } else {
        pwm6step_set_direction(CLOCKWISE);
    }

    // public function was called, set control mode to duty direct
    motor_control_mode = DUTY;

    uint16_t duty_cycle_abs = abs(duty_cycle);
    set_duty_cycle(duty_cycle_abs);
}

void pwm6step_set_duty_cycle_f(float duty_cycle_pct) {
    if (duty_cycle_pct < 0) {
        pwm6step_set_direction(COUNTER_CLOCKWISE);
    } else {
        pwm6step_set_direction(CLOCKWISE);
    }

    pwm6step_set_duty_cycle((uint16_t) (duty_cycle_pct * (float) MAX_DUTYCYCLE_COMMAND));
}

static void set_voltage(uint16_t voltage_mv) {
    // we can't command more than the battery can currently offer
    // set it to the max we can offer (gets more effective PWM range
    // at the top end)
    if (voltage_mv > measured_vbus_voltage) {
        voltage_mv = measured_vbus_voltage;
    }

    last_voltage_command_mv = voltage_mv;

    // scale from mv to max duty
    uint32_t voltage_scaled_to_dc = (uint16_t) ((uint32_t) voltage_mv * MAX_DUTYCYCLE_COMMAND / (uint32_t) measured_vbus_voltage);
    pwm6step_set_duty_cycle(voltage_scaled_to_dc);
}

void pwm6step_set_voltage(int16_t voltage_mv) {
    if (voltage_mv < 0) {
        pwm6step_set_direction(COUNTER_CLOCKWISE);
    } else {
        pwm6step_set_direction(CLOCKWISE);
    }

    // public function was called, set control mode to voltage
    motor_control_mode = VOLTAGE;

    uint16_t abs_voltage_mv = abs(voltage_mv);
    set_voltage(abs_voltage_mv);
}

static void set_current(uint16_t current_ma) {
    // map current_ma to 4096
    Uint16FixedPoint_t scaled_current = (current_ma * S0F16_4096_OVER_9000) >> 16;
    fxptpi_setpoint(&current_controller, (Int16FixedPoint_t) scaled_current);
}

void pwm6step_set_current(int16_t current_ma) {
    if (current_ma < 0) {
        pwm6step_set_direction(COUNTER_CLOCKWISE);
    } else {
        pwm6step_set_direction(CLOCKWISE);
    }

    motor_control_mode = CURRENT;

    uint16_t abs_current_ma = abs(current_ma);
    set_current(abs_current_ma);
}

bool pwm6step_1ms_flag() {
    if (flag_1ms) {
        flag_1ms = false;
        return true;
    }

    return false;
}


bool pwm6step_hall_rps_estimate_valid() {
    return hall_speed_estimate_valid;
}

int pwm6step_hall_get_rps_estimate() {
    return 0;
}

const MotorErrors_t pwm6step_get_motor_errors() {
    return motor_errors;
} 

const uint16_t pwm6step_get_current_measurement() {
    return measured_current;
}

const uint16_t* pwm6step_get_current_log() {
    // use the inactive buffer
    return current_data_buffer[!double_buffer_ind];
}

const uint16_t pwm6step_get_vbus_voltage() {
    return measured_vbus_voltage;
}  

const uint16_t pwm6step_get_voltage_command() {
    return last_voltage_command_mv;
}

