/**
 * @brief 
 * 
 * @param timer 
 */

#pragma once

#include <stm32f031x6.h>

///////////////////////////////////
//  public function definitions  //
///////////////////////////////////

void quadenc_setup();
uint16_t quadenc_get_counter();
void quadenc_reset_encoder_delta();
int32_t quadenc_get_encoder_delta();
