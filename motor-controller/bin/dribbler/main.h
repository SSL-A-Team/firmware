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

#pragma once

// expect cmd packets at 100H Hz (10ms), ticks are currently 1ms per
// timeout ticks equates to 10 consecutive missed packets.
#define COMMAND_PACKET_TIMEOUT_TICKS 100

#define AVDD_V 3.3f

///////////////////////////////////////////
//  current sense amplification network  //
///////////////////////////////////////////

#define CS_AMP_NETWORK_V_TO_I_LINEAR_M 1.432f
#define CS_AMP_NETWORK_V_TO_I_LINEAR_B 0.252f

///////////////////
//  motor model  //
///////////////////

#define ENC_TICKS_PER_REV 4000
#define HALL_TRANSITIONS_PER_REV 48
#define MAX_MOTOR_RPM 5260
#define MAX_MOTOR_REV_PER_S (5260.0f / 60.0f)

////////////////////////
//  FILTERING/TUNING  //
////////////////////////

#define ENCODER_IIR_TF_MS 0.20f
#define TORQUE_IIR_TF_MS 0.40f
// #define DC_IIR_TF_MS 0.20f