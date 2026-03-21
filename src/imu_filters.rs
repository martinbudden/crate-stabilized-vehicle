use cfg_if::cfg_if;
use filters::{BiquadFilterf32, FilterPt1f32};
use imu_sensors::ImuReadingf32;
use vector_quaternion_matrix::Vector3df32;

#[cfg(feature = "use_rpm_filters")]
use motor_mixers::RpmFiltersState;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuFiltersConfig {
    pub acc_lpf_hz: u16,
    pub gyro_lpf1_hz: u16,
    pub gyro_lpf2_hz: u16,
    pub gyro_notch1_hz: u16,
    pub gyro_notch1_cutoff: u16,
    pub gyro_notch2_hz: u16,
    pub gyro_notch2_cutoff: u16,
}

impl ImuFiltersConfig {
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

impl Default for ImuFiltersConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuFiltersState {
    motor_count: usize,
    config: ImuFiltersConfig,
    acc_lpf: FilterPt1f32<Vector3df32>,
    gyro_lpf1: FilterPt1f32<Vector3df32>,
    gyro_lpf2: FilterPt1f32<Vector3df32>,
    gyro_notch1: BiquadFilterf32<Vector3df32>,
    gyro_notch2: BiquadFilterf32<Vector3df32>,
    #[cfg(feature = "use_rpm_filters")]
    rpm_filters: RpmFiltersState,
}

impl ImuFiltersState {
    pub fn new() -> Self {
        Self {
            motor_count: 4,
            config: ImuFiltersConfig::default(),
            acc_lpf: FilterPt1f32::default(),
            gyro_lpf1: FilterPt1f32::default(),
            gyro_lpf2: FilterPt1f32::default(),
            gyro_notch1: BiquadFilterf32::default(),
            gyro_notch2: BiquadFilterf32::default(),
            #[cfg(feature = "use_rpm_filters")]
            rpm_filters: RpmFiltersState::default(),
        }
    }
    pub fn set_config(&mut self, config: ImuFiltersConfig, delta_t: f32) {
        self.config = config;
        self.acc_lpf.set_cutoff_frequency_and_reset(config.acc_lpf_hz as f32, delta_t);
        self.gyro_lpf1.set_cutoff_frequency_and_reset(config.gyro_lpf1_hz as f32, delta_t);
        self.gyro_lpf2.set_cutoff_frequency_and_reset(config.gyro_lpf1_hz as f32, delta_t);
        self.gyro_notch1.set_notch_frequency(config.gyro_notch1_hz as f32, config.gyro_notch1_cutoff as f32);
        self.gyro_notch2.set_notch_frequency(config.gyro_notch2_hz as f32, config.gyro_notch2_cutoff as f32);
    }
}

impl Default for ImuFiltersState {
    fn default() -> Self {
        Self::new()
    }
}

pub trait ImuFilters {
    fn state(&self) -> &ImuFiltersState;
    fn state_mut(&mut self) -> &mut ImuFiltersState;
    fn config(&self) -> &ImuFiltersConfig;

    fn filter(&mut self, imu_reading: ImuReadingf32, delta_t: f32) -> ImuReadingf32;
}

impl ImuFilters for ImuFiltersState {
    fn state(&self) -> &ImuFiltersState {
        self
    }
    fn state_mut(&mut self) -> &mut ImuFiltersState {
        self
    }
    fn config(&self) -> &ImuFiltersConfig {
        &self.state().config
    }

    fn filter(&mut self, mut imu_reading: ImuReadingf32, _delta_t: f32) -> ImuReadingf32 {
        if self.config().acc_lpf_hz != 0 {
            imu_reading.acc = self.state_mut().acc_lpf.filter(imu_reading.acc);
        }
        if self.config().gyro_lpf1_hz != 0 {
            imu_reading.gyro_rps = self.state_mut().gyro_lpf1.filter(imu_reading.gyro_rps);
        }
        if self.config().gyro_lpf2_hz != 0 {
            imu_reading.gyro_rps = self.state_mut().gyro_lpf2.filter(imu_reading.gyro_rps);
        }
        if self.config().gyro_notch1_hz != 0 {
            imu_reading.gyro_rps = self.state_mut().gyro_notch1.filter(imu_reading.gyro_rps);
        }
        if self.config().gyro_notch2_hz != 0 {
            imu_reading.gyro_rps = self.state_mut().gyro_notch2.filter(imu_reading.gyro_rps);
        }
        #[cfg(feature = "use_rpm_filters")]

        cfg_if! {
            if #[cfg(feature = "use_rpm_filters")] {
            for ii in 0..self.state().motor_count {
                imu_reading.gyro_rps = self.state_mut().rpm_filters.filter(imu_reading.gyro_rps,ii);
            }
            }
        }

        imu_reading
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn is_normal<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_normal::<ImuFiltersConfig>();
        is_normal::<ImuFiltersState>();
    }
    #[test]
    fn new() {
        let config = ImuFiltersConfig::new();
        assert_eq!(100, config.acc_lpf_hz);
    }
}
