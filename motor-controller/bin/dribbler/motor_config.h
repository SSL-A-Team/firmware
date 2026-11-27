#pragma once

// TODO: verify direction for dribbler motor hardware
#define MOTOR_POLE_PAIRS 1U
#define INVERT_MOTOR_DIRECTION false

// Model-based current observer constants (see CURRENT_SENSING_INVESTIGATION.md).
// TODO: the dribbler motor has not been characterized. These are the wheel
// (Nanotec DF45) values carried over as placeholders, so current_model_ma in
// dribbler telemetry is not calibrated and should not be trusted for torque
// work until Ke and the winding resistance are measured.
#define MOTOR_KE_MV_PER_DRAD_Q8 858U
#define MOTOR_R_LOOP_MOHM 880U
