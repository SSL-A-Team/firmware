#pragma once

#include <stdbool.h>

typedef enum SchmidtTriggerState {
    SCHMIDT_TRIGGER_STATE_ACTIVE_LOW,
    SCHMIDT_TRIGGER_STATE_ACTIVE_HIGH,
    SCHMIDT_TRIGGER_STATE_INACTIVE
} SchmidtTriggerState_t;

typedef struct SchmidtTrigger {
    SchmidtTriggerState_t state;
    float hysteresis_low;
    float hysteresis_high;
} SchmidtTrigger_t;

typedef struct CenterSchmidtTrigger {
    SchmidtTriggerState_t state;
    float hysteresis_low_active;
    float hysteresis_low_inactive;
    float hysteresis_high_inactive;
    float hysteresis_high_active;
    float inactive_value;
} CenterSchmidtTrigger_t;

void schtrig_init(SchmidtTrigger_t *st, float h_low, float h_high);
float schtrig_update(SchmidtTrigger_t *st, float val);
void schtrigc_init(CenterSchmidtTrigger_t *cst, float h_low_active, float h_low_inactive, float h_high_inactive, float h_high_active, float i_value);
float schtrigc_update(CenterSchmidtTrigger_t *cst, float val);