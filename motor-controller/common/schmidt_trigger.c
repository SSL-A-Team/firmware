
#include "schmidt_trigger.h"

void schtrig_init(SchmidtTrigger_t *st, float h_low, float h_high) {
    st->state = SCHMIDT_TRIGGER_STATE_INACTIVE;
    st->hysteresis_low = h_low;
    st->hysteresis_high = h_high;
}

float schtrig_update(SchmidtTrigger_t *st, float val) {
    if (st->state == SCHMIDT_TRIGGER_STATE_INACTIVE && val > st->hysteresis_high) {
        st->state = SCHMIDT_TRIGGER_STATE_ACTIVE_HIGH;
    }

    if (st->state == SCHMIDT_TRIGGER_STATE_ACTIVE_HIGH && val < st->hysteresis_low) {
        st->state = SCHMIDT_TRIGGER_STATE_INACTIVE;
    }

    if (st->state == SCHMIDT_TRIGGER_STATE_INACTIVE) {
        return 0.0f;
    }

    return val;
}

void schtrigc_init(CenterSchmidtTrigger_t *cst, float h_low_active, float h_low_inactive, float h_high_inactive, float h_high_active, float i_value) {
        cst->state = SCHMIDT_TRIGGER_STATE_INACTIVE;
        cst->hysteresis_low_active = h_low_active;
        cst->hysteresis_low_inactive = h_low_inactive;
        cst->hysteresis_high_inactive = h_high_inactive;
        cst->hysteresis_high_active = h_high_active;
        cst->inactive_value = i_value;
}

float schtrigc_update(CenterSchmidtTrigger_t *cst, float val) {
    if (cst->hysteresis_low_inactive < val && val < cst->hysteresis_high_inactive) {
        cst->state = SCHMIDT_TRIGGER_STATE_INACTIVE;
    }

    if (val < cst->hysteresis_low_active) {
        cst->state = SCHMIDT_TRIGGER_STATE_ACTIVE_LOW;
    }

    if (val > cst->hysteresis_high_active) {
        cst->state = SCHMIDT_TRIGGER_STATE_ACTIVE_HIGH;
    }

    if (cst->state == SCHMIDT_TRIGGER_STATE_INACTIVE) {
        return cst->inactive_value;
    }

    return val;
}