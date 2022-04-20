
#include <stm32f031x6.h>

/**
 * @brief blocks for a number of milliseconds. Assumes SysTick is already running
 * 
 * @param time_ms time, in milliseconds, to block
 */
void wait_ms(uint32_t time_ms);