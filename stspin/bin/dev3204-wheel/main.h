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

#define IOQ_BUF_LEN 64

///////////////////
//  motor model  //
///////////////////

#define ENC_TICKS_PER_REV 4000
#define HALL_TRANSITIONS_PER_REV 48
#define MAX_MOTOR_RPM 5260
#define MAX_MOTOR_REV_PER_S (5260.0f / 60.0f)