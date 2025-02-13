/**
 * @brief 
 * 
 * @param timer 
 */

#pragma once

#include <stm32f031x6.h>

#include "system.h"

///////////////////////////////////
//  public function definitions  //
///////////////////////////////////

void quadenc_setup();
void quadenc_reset_encoder_delta();
int32_t quadenc_get_encoder_delta();
float quadenc_delta_to_w(float encoder_delta, float deltat_s);
float rpm_to_rad_s(float rpm);
