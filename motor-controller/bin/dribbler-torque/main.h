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

// expect cmd packets at 100Hz (10ms), ticks are currently 1ms per
// timeout ticks equates to 10 consecutive missed packets.
#define COMMAND_PACKET_TIMEOUT_TICKS 100

#define AVDD_V 3.0f

///////////////////////////////////////////
//  current sense amplification network  //
///////////////////////////////////////////

#define CS_AMP_NETWORK_V_TO_I_LINEAR_M 1.432f
#define CS_AMP_NETWORK_V_TO_I_LINEAR_B 0.252f

////////////////////////
//  FILTERING/TUNING  //
////////////////////////

// hall velocity IIR low-pass cutoff
#define HALL_VEL_FILTER_CUTOFF_HZ 20.0f

// ECU22048H24-S101: rated 1.67A (1670mA), peak 2.94A (2940mA)
// threshold in decirads/s above which peak current is allowed
#define DRIBBLER_TURNING_VEL_THRESH_DRADS 3
// 90% of rated continuous current in mA
#define MAX_CURR_DRIBBLER_NOT_TURNING 1503
// 90% of peak current in mA
#define MAX_CURR_DRIBBLER_TURNING 2646
