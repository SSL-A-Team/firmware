
use embassy_stm32::adc::{self, Adc, AdcChannel, Resolution, SampleTime, VrefInt, VREF_DEFAULT_MV};
use embassy_stm32::Peripheral;

pub struct AdcHelper<'a, T: adc::Instance, Ch: AdcChannel<T>> where VrefInt: AdcChannel<T> {
    inst: Adc<'a, T>,
    channel_pin: Ch,
    channel_vref: VrefInt
}

impl<'a, T: adc::Instance, Ch: AdcChannel<T>> AdcHelper<'a, T, Ch> 
where VrefInt: AdcChannel<T> {
    pub fn new(
        peri: impl Peripheral<P = T> + 'a,
        channel_pin: Ch,
        sample_time: SampleTime,
        resolution: Resolution
    ) -> Self {

        let mut adc_inst = Adc::new(peri);

        adc_inst.set_sample_time(sample_time); 
        adc_inst.set_resolution(resolution);

        let channel_vref = adc_inst.enable_vrefint();

        AdcHelper {
            inst: adc_inst,
            channel_pin: channel_pin,
            channel_vref: channel_vref
        }
    }

    pub fn read_f32(&mut self) -> f32 {
        // Gets the Vref since it changes based on chip class.
        let vref_cur_mv = self.inst.read(&mut self.channel_vref) as f32;
        // Scale by Vref to convert to absolute voltage.
        return ((VREF_DEFAULT_MV as f32) / vref_cur_mv) * (self.inst.read(&mut self.channel_pin) as f32);
    }
}