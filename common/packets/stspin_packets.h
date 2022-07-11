/**
 * @file motor.h
 * @author Will Stuckey
 * @brief 
 * @version 0.1
 * @date 2022-06-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <stdint.h>

typedef float PidValue_t;

typedef enum MotorCommandPacketType {
    MCP_PARAMS,
    MCP_MOTION
} MotorCommandPacketType_t;

typedef struct MotorCommand_Params_Packet {
    uint16_t update_timestamp : 1;
    uint16_t update_vel_p : 1;
    uint16_t update_vel_i : 1;
    uint16_t update_vel_d : 1;
    uint16_t update_cur_P : 1;
    uint16_t update_cur_i : 1;
    uint16_t update_cur_d : 1;
    uint16_t update_cur_clamp : 1;
    // add future flags here, decrement reserved
    uint16_t reserved : 8;

    uint32_t timestamp;
    PidValue_t vel_p;
    PidValue_t vel_i;
    PidValue_t vel_d;
    PidValue_t cur_p;
    PidValue_t cur_i;
    PidValue_t cur_d;
    uint16_t cur_clamp;
    // add future params here
} MotorCommand_Params_Packet_t;

typedef enum MotorCommand_MotionType {
    OPEN_LOOP = 0,
    VELOCITY = 1,
    TORQUE = 2,
    BOTH= 3
} MotorCommand_MotionType_t;

typedef struct MotorCommand_Motion_Packet {
    uint16_t reset : 1;
    MotorCommand_MotionType_t motion_control_type : 2;

    int16_t setpoint;
} MotorCommand_Motion_Packet_t;

typedef struct MotorCommandPacket {
    MotorCommandPacketType_t type;
    uint32_t crc32;
    union {
        MotorCommand_Params_Packet_t params;
        MotorCommand_Motion_Packet_t motion;
    };
} MotorCommandPacket_t;

/////////////////
//  responses  //
/////////////////

typedef enum MotorResponsePacketType {
    MRP_PARAMS,
    MRP_MOTION
} MotorResponsePacketType_t;

typedef struct MotorResponse_Params_Packet {
    uint8_t version_major;
    uint8_t version_minor;
    uint16_t version_patch;
    uint32_t timestamp;
    PidValue_t vel_p;
    PidValue_t vel_i;
    PidValue_t vel_d;
    PidValue_t cur_p;
    PidValue_t cur_i;
    PidValue_t cur_d;
    uint16_t cur_clamp;
} MotorResponse_Params_Packet_t;

typedef struct MotorResponse_Motion_Packet {
    uint16_t master_error : 1;
    uint16_t hall_power_error : 1;
    uint16_t hall_disconnected_error : 1;
    uint16_t bldc_transition_error : 1;
    uint16_t bldc_commutation_watchdog_error : 1;
    uint16_t enc_disconnected_error: 1;
    uint16_t enc_decoding_error : 1;
    uint16_t hall_enc_vel_disagreement_error: 1;
    uint16_t overcurrent_error : 1;
    uint16_t undervoltage_error : 1;
    uint16_t overvoltage_error : 1;
    uint16_t torque_limited : 1;
    uint16_t control_loop_time_error: 1;
    uint16_t reserved : 3;

    uint32_t timestamp;
    uint16_t encoder_delta;
    float vel_enc_estimate;
    float vel_hall_estimate;
    float vel_computed_error;
    float vel_computed_setpoint;
    float current_estimate;
    float current_computed_error;
    float current_computed_setpoint;
} MotorResponse_Motion_Packet_t;

typedef struct MotorResponsePacket {
    MotorResponsePacketType_t type;
    uint32_t crc32;
    union {
        MotorResponse_Params_Packet_t params;
        MotorResponse_Motion_Packet_t motion;
    };
} MotorResponsePacket_t;