use embassy_stm32::adc::{self, Adc, AdcChannel, Resolution, SampleTime};

// The voltage which the internal ADC were calibrated at.
// For the H743 and F407
const V_CAL_V: f32 = 3.3;

pub struct AdcHelper<'a, T: adc::DefaultInstance, Ch: AdcChannel<T>> {
    inst: Adc<'a, T>,
    pin: Ch,
    sample_time: SampleTime,
    adc_bins: u32,
}

impl<'a, T: adc::DefaultInstance, Ch: AdcChannel<T>> AdcHelper<'a, T, Ch> {
    // The ADC's constructor is chip-specific (embassy-stm32's g4 HAL, for example,
    // has a differently-shaped `new` than the v2/v3/v4 HAL versions), so callers
    // construct the `Adc` themselves and hand it in already configured with `resolution`.
    pub fn new(
        adc_inst: Adc<'a, T>,
        pin: Ch,
        sample_time: SampleTime,
        resolution: Resolution,
    ) -> Self {
        // Use resolution to calculate the max ADC min quantity.'
        let res_bits: u32 = match resolution {
            Resolution::BITS12 => 12,
            Resolution::BITS10 => 10,
            Resolution::BITS8 => 8,
            _ => 0,
        };
        let adc_bins = u32::pow(2, res_bits) - 1;
        AdcHelper {
            inst: adc_inst,
            pin,
            sample_time,
            adc_bins,
        }
    }

    // vref_int_read_mv has to be passed in because the ADC peripheral that
    // it is connected to depends on the chip.
    pub fn read_volt_raw_f32(&mut self, _vref_int_read_mv: f32, _vref_int_cal: f32) -> f32 {
        // Based off of this: http://www.efton.sk/STM32/STM32_VREF.pdf
        // vmeas = vcal * MEAS / MAX * CAL / REFINT (4)

        // defmt::info!("V_CAL_MV: {}", V_CAL_V);
        // defmt::info!("inst.read(): {}", self.inst.read(&mut self.pin) as f32);
        // defmt::info!("adc_bins: {}", self.adc_bins as f32);
        // defmt::info!("vref_int_cal: {}", vref_int_cal);
        // defmt::info!("vref_int_read_mv: {}", vref_int_read_mv);

        V_CAL_V * (self.inst.blocking_read(&mut self.pin, self.sample_time) as f32)
            / (self.adc_bins as f32)
        // * vref_int_cal / vref_int_read_mv;
    }
}
