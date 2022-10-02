#include <stm32f031x6.h>

#include <limits.h>

#include "6step.h"
#include "current_sensing.h"
#include "debug.h"
#include "iir.h"
#include "main.h"
#include "schmidt_trigger.h"
#include "setup.h"
#include "system.h"
#include "time.h"

__attribute__((optimize("O0")))
int main() {
    setup();

    GPIOB->BSRR |= GPIO_BSRR_BR_8;
    GPIOB->BSRR |= GPIO_BSRR_BR_9;

    // initialize 
    ADC_Result_t res;
    currsen_setup(ADC_MODE, &res, ADC_NUM_CHANNELS, ADC_CH_MASK, ADC_SR_MASK);

    pwm6step_setup();
    pwm6step_set_duty_cycle_f(0.0f);

    // enable ADC hardware trigger (tied to 6step timer)
    currsen_enable_ht();

    IIRFilter_t trimpot_filter;
    iir_filter_init(&trimpot_filter, iir_filter_alpha_from_Tf(ADC_IIR_TF_MS, LOOP_RATE_MS));

    CenterSchmidtTrigger_t pot_trig;
    schtrigc_init(&pot_trig, 0.44, 0.48, 0.52, 0.56, 0.50);

    while (true) {
        uint16_t pot_val = res.V_pot;
        float pot_pct = (float) pot_val / (float) UINT16_MAX;
        float pot_pct_filt = iir_filter_update(&trimpot_filter, pot_pct);
        pot_pct_filt = schtrigc_update(&pot_trig, pot_pct_filt);
        if (pot_pct_filt < 0.47f) {
            float adj = -((0.50f - pot_pct_filt) * 2.0f);
            pwm6step_set_duty_cycle_f(adj);
        } else if (pot_pct_filt > 0.53f) {
            float adj = (pot_pct_filt - 0.50f) * 2.0f;
            pwm6step_set_duty_cycle_f(adj);
        } else {
            pwm6step_set_duty_cycle_f(0.0f);
        }

        sync_systick();
    }
}
