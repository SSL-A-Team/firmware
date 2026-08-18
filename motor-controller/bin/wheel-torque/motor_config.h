#pragma once

#define MOTOR_POLE_PAIRS 8U
#define INVERT_MOTOR_DIRECTION false

// Model-based current observer constants (see CURRENT_SENSING_INVESTIGATION.md).
// Nanotec DF45M024053-A2: Ke = 0.0335 V*s/rad = 3.35 mV per deci-rad/s.
// 3.35 * 256 = 858 in UQ8.8.
#define MOTOR_KE_MV_PER_DRAD_Q8 858U
// Total loop resistance during the active vector:
//   0.800 windings (line-to-line) + 2 * 0.017 STL8N10F7 R_DS(on) + 0.050 shunt
// Rounded to 0.88 ohm. Note this drifts with winding temperature
// (copper +0.39%/degC), which the observer does not currently compensate.
#define MOTOR_R_LOOP_MOHM 880U
