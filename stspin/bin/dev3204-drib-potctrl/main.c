#include <stm32f031x6.h>

#include <limits.h>

#include "6step.h"
#include "current_sensing.h"
#include "main.h"
#include "setup.h"
#include "system.h"
#include "time.h"

__attribute__((optimize("O0")))
int main() {
    setup();

    GPIOB->BSRR |= GPIO_BSRR_BR_8;
    GPIOB->BSRR |= GPIO_BSRR_BR_9;

    pwm6step_setup();

    ADC_Result_t res;
    currsen_setup();

    const uint16_t UINT16_HALF_VAL = UINT16_MAX / 2;
    const uint16_t UINT16_5PCT = UINT16_MAX / 20;

    // toggle J1-1
    while (true) {

        currsen_read(&res);
        uint16_t pot_val = res.pot;

        if (pot_val < UINT16_HALF_VAL - UINT16_5PCT) {
            pwm6step_set_duty_cycle(-((int32_t) (UINT16_HALF_VAL - pot_val) * 2));
        } else if (pot_val > UINT16_HALF_VAL + UINT16_5PCT) {
            pwm6step_set_duty_cycle((pot_val - UINT16_HALF_VAL) * 2);
        } else {
            pwm6step_set_duty_cycle(0);
        }

        sync_systick();
    }
}
