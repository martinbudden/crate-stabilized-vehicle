use cfg_if::cfg_if;
use filters::{BiquadFilterf32, FilterSignal, Pt1Filterf32};
use imu_sensors::ImuReadingf32;
use motor_mixers::RpmFilters;
use vector_quaternion_matrix::Vector3df32;

#[cfg(feature = "use_rpm_filters")]
use motor_mixers::RpmFilterBank;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuFilterBankConfig {
    pub acc_lpf_hz: u16,
    pub gyro_lpf1_hz: u16,
    pub gyro_lpf2_hz: u16,
    pub gyro_notch1_hz: u16,
    pub gyro_notch1_cutoff: u16,
    pub gyro_notch2_hz: u16,
    pub gyro_notch2_cutoff: u16,
}

impl ImuFilterBankConfig {
    pub fn new() -> Self {
        Self {
            acc_lpf_hz: 100,
            gyro_lpf1_hz: 0,   // switched off
            gyro_lpf2_hz: 250, // this is an anti-alias filter and shouldn't be disabled
            gyro_notch1_hz: 0,
            gyro_notch1_cutoff: 0,
            gyro_notch2_hz: 0,
            gyro_notch2_cutoff: 0,
        }
    }
}

impl Default for ImuFilterBankConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuFilterBank {
    motor_count: usize,
    config: ImuFilterBankConfig,
    acc_lpf: Pt1Filterf32<Vector3df32>,
    gyro_lpf1: Pt1Filterf32<Vector3df32>,
    gyro_lpf2: Pt1Filterf32<Vector3df32>,
    gyro_notch1: BiquadFilterf32<Vector3df32>,
    gyro_notch2: BiquadFilterf32<Vector3df32>,
    #[cfg(feature = "use_rpm_filters")]
    rpm_filters: RpmFilterBank,
}

impl Default for ImuFilterBank {
    fn default() -> Self {
        Self::new()
    }
}

impl ImuFilterBank {
    pub fn new() -> Self {
        Self {
            motor_count: 4,
            config: ImuFilterBankConfig::default(),
            acc_lpf: Pt1Filterf32::default(),
            gyro_lpf1: Pt1Filterf32::default(),
            gyro_lpf2: Pt1Filterf32::default(),
            gyro_notch1: BiquadFilterf32::default(),
            gyro_notch2: BiquadFilterf32::default(),
            #[cfg(feature = "use_rpm_filters")]
            rpm_filters: RpmFilterBank::default(),
        }
    }
    pub fn set_config(&mut self, config: ImuFilterBankConfig, delta_t: f32) {
        self.config = config;
        self.acc_lpf.set_cutoff_frequency_and_reset(config.acc_lpf_hz as f32, delta_t);
        self.gyro_lpf1.set_cutoff_frequency_and_reset(config.gyro_lpf1_hz as f32, delta_t);
        self.gyro_lpf2.set_cutoff_frequency_and_reset(config.gyro_lpf1_hz as f32, delta_t);
        self.gyro_notch1.set_notch_frequency(config.gyro_notch1_hz as f32, config.gyro_notch1_cutoff as f32);
        self.gyro_notch2.set_notch_frequency(config.gyro_notch2_hz as f32, config.gyro_notch2_cutoff as f32);
    }
}

pub trait FilterImuReading {
    fn state(&self) -> &ImuFilterBank;
    fn state_mut(&mut self) -> &mut ImuFilterBank;
    fn config(&self) -> &ImuFilterBankConfig;

    fn apply(&mut self, imu_reading: ImuReadingf32, delta_t: f32) -> ImuReadingf32;
}

impl FilterImuReading for ImuFilterBank {
    fn state(&self) -> &ImuFilterBank {
        self
    }
    fn state_mut(&mut self) -> &mut ImuFilterBank {
        self
    }
    fn config(&self) -> &ImuFilterBankConfig {
        &self.state().config
    }

    fn apply(&mut self, mut imu_reading: ImuReadingf32, _delta_t: f32) -> ImuReadingf32 {
        if self.config().acc_lpf_hz != 0 {
            imu_reading.acc = self.state_mut().acc_lpf.apply(imu_reading.acc);
        }
        if self.config().gyro_lpf1_hz != 0 {
            imu_reading.gyro_rps = self.state_mut().gyro_lpf1.apply(imu_reading.gyro_rps);
        }
        if self.config().gyro_lpf2_hz != 0 {
            imu_reading.gyro_rps = self.state_mut().gyro_lpf2.apply(imu_reading.gyro_rps);
        }
        if self.config().gyro_notch1_hz != 0 {
            imu_reading.gyro_rps = self.state_mut().gyro_notch1.apply(imu_reading.gyro_rps);
        }
        if self.config().gyro_notch2_hz != 0 {
            imu_reading.gyro_rps = self.state_mut().gyro_notch2.apply(imu_reading.gyro_rps);
        }
        #[cfg(feature = "use_rpm_filters")]

        cfg_if! {
            if #[cfg(feature = "use_rpm_filters")] {
            for ii in 0..self.state().motor_count {
                imu_reading.gyro_rps = self.state_mut().rpm_filters.apply_notch_filters(imu_reading.gyro_rps,ii);
            }
            }
        }

        imu_reading
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<ImuFilterBankConfig>();
        is_full::<ImuFilterBank>();
    }
    #[test]
    fn new() {
        let config = ImuFilterBankConfig::new();
        assert_eq!(100, config.acc_lpf_hz);
    }
}
