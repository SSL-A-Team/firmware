use crate::units::{MV_PER_V, V_PER_MV};

pub struct AdcConverter {
    use_vref_int: bool,
    use_temp: bool,
    vref_ext_mv: u16,
    vref_int_mv: u16,
    last_vrefint: u16,
}

impl AdcConverter {
    pub const fn new(vref_ext_mv: u16, vref_int_mv: u16, use_vref_int: bool, use_temp: bool) -> Self {
        AdcConverter {
            use_vref_int,
            use_temp,
            vref_ext_mv, 
            vref_int_mv, 
            last_vrefint: 0,
        }
    }

    pub fn update_vrefint(&mut self, vrefint_sample: u16) {
        self.last_vrefint = vrefint_sample
    }

    fn raw_sample_to_mv_no_comp(&self, sample: u16) -> u16 {
        ((sample as u32 / 4096) * (self.vref_ext_mv as u32) * (MV_PER_V as u32)) as u16
    }

    fn raw_sample_to_mv_vrefint_comp(&self, sample: u16) -> u16 {    
        (u32::from(sample) * u32::from(self.vref_int_mv) / u32::from(self.last_vrefint)) as u16
    }

    pub fn raw_sample_to_mv(&self, sample: u16) -> u16 {
        let sample_mv = if self.use_vref_int {
            self.raw_sample_to_mv_vrefint_comp(sample)
        } else {
            self.raw_sample_to_mv_no_comp(sample)
        };

        if self.use_temp {
            unimplemented!()
        }

        sample_mv
    }

    pub fn raw_sample_to_v(&self, sample: u16) -> f32 {
        self.raw_sample_to_mv(sample) as f32 * V_PER_MV
    }

    pub fn raw_samples_to_mv(&self, raw_samples: &[u16], mv_samples: &mut [u16]) {
        for (mv_sample, raw_sample) in mv_samples.iter_mut().zip(raw_samples) {
            *mv_sample = self.raw_sample_to_mv(*raw_sample);
        }
    }

    pub fn raw_samples_to_mv_inplace(&self, raw_samples: &mut [u16]) {
        for sample in raw_samples.iter_mut() {
            *sample = self.raw_sample_to_mv(*sample);
        }
    }

    pub fn raw_samples_to_v(&self, raw_samples: &[u16], v_samples: &mut [f32]) {
        for (mv_sample, raw_sample) in v_samples.iter_mut().zip(raw_samples) {
            *mv_sample = self.raw_sample_to_v(*raw_sample);
        }
    }
}