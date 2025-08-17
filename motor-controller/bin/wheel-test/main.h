/**
 * @file main.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <math.h>

#pragma once

// version
#define VERSION_MAJOR 0
#define VERSION_MINOR 1
#define VERSION_PATCH 0

// expect cmd packets at 100H Hz (10ms), ticks are currently 1ms per
// timeout ticks equates to 10 consecutive missed packets.
#define COMMAND_PACKET_TIMEOUT_TICKS 100
#define IOQ_BUF_LEN 64

#define AVDD_V 3.3f

///////////////////////////////////////////
//  current sense amplification network  //
///////////////////////////////////////////

#define CS_AMP_NETWORK_V_TO_I_LINEAR_M 1.432f
#define CS_AMP_NETWORK_V_TO_I_LINEAR_B 0.252f

///////////////////
//  motor model  //
///////////////////

#define DF45_ENC_TICKS_PER_REV 4000
#define DF45_HALL_TRANSITIONS_PER_REV 48
#define DF45_MAX_MOTOR_ROT_PER_M 5260
#define DF45_MAX_MOTOR_ROT_PER_S ((float) DF45_MAX_MOTOR_ROT_PER_M / 60.0f)
#define DF45_MAX_MOTOR_RAD_PER_S (DF45_MAX_MOTOR_ROT_PER_S * 2.0f * (float) M_PI)
#define DF45_RATED_CURRENT 2.36f
#define DF45_TORQUE_TO_CURRENT_LINEAR_M 28.690f
#define DF45_TORQUE_TO_CURRENT_LINEAR_B (-0.0558f)
#define DF45_CURRENT_TO_TORQUE_LINEAR_M 0.03477f
#define DF45_CURRENT_TO_TORQUE_LINEAR_B 0.00242f
#define DF45_RADS_TO_DC_LINEAR_M 0.00144873f
#define DF45_RADS_TO_DC_LINEAR_B 0.0057431f

////////////////////////
//  FILTERING/TUNING  //
////////////////////////

#define ENCODER_IIR_TF_MS 0.20f
#define TORQUE_IIR_TF_MS 0.40f
// #define DC_IIR_TF_MS 0.20f