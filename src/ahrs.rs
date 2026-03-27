//#![allow(unused)]
use crate::{FilterImuReading, ImuFilterBank, VehicleController};
use imu_sensors::{ImuReading, ImuReadingf32};
use sensor_fusion::SensorFusion;
use vector_quaternion_matrix::{Quaternion, Quaternionf32, Vector3df32};

#[cfg(feature = "use_complementary_filter")]
use sensor_fusion::ComplementaryFilterf32;
#[cfg(feature = "use_madgwick_filter")]
use sensor_fusion::MadgwickFilterf32;
#[cfg(feature = "use_mahony_filter")]
use sensor_fusion::MahonyFilterf32;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AhrsData {
    // AhrsMessageQueue contains this data
    imu_reading: ImuReadingf32,
    gyro_rps_unfiltered: Vector3df32,
    orientation: Quaternionf32,
    delta_t: f32,
    time_microseconds: u32,
}

pub trait Ahrs {
    fn state(&self) -> &AhrsState;
    fn state_mut(&mut self) -> &mut AhrsState;
    //fn config(&self) -> &AhrsConfig;

    fn read_imu_and_update_orientation(
        &mut self,
        time_us: u32,
        imu_filters: ImuFilterBank,
        vehicle_controller: VehicleController,
    ) -> AhrsData;

    fn set_overflow_sign_change_threshold_rps(&mut self, overflow_sign_change_threshold_rps: f32) {
        self.state_mut().overflow_sign_change_threshold_rps_squared =
            overflow_sign_change_threshold_rps * overflow_sign_change_threshold_rps;
    }
    // Check for overflow on z axis, ie sign of z-value has changed when the z-value is large
    fn check_gyro_overflow_z(&mut self) {
        if self.state().ahrs_data.imu_reading.gyro_rps.z * self.state().gyro_rps_previous.z
            < -self.state().overflow_sign_change_threshold_rps_squared
        {
            // we've had a sign change of a large value, ie from (say) 1900 to -1950, so this is an overflow, so don't accept the new gyro z-value
            self.state_mut().ahrs_data.imu_reading.gyro_rps.z = self.state().gyro_rps_previous.z;
        } else {
            // normal sign change, ie from (say) 20 to -10, so set _gyro_rps_previous for next time round
            self.state_mut().gyro_rps_previous.z = self.state().ahrs_data.imu_reading.gyro_rps.z;
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AhrsState {
    imu_filters: ImuFilterBank,
    overflow_sign_change_threshold_rps_squared: f32,
    gyro_rps_previous: Vector3df32, // For overflow checking
    ahrs_data: AhrsData,
    sensor_fusion_filter_is_initializing: bool,
    #[cfg(feature = "use_madgwick_filter")]
    sensor_fusion_filter: MadgwickFilterf32,
    #[cfg(feature = "use_mahony_filter")]
    sensor_fusion_filter: MahonyFilterf32,
    #[cfg(feature = "use_complementary_filter")]
    sensor_fusion_filter: ComplementaryFilterf32,
}

impl AhrsState {
    fn new() -> Self {
        Self {
            imu_filters: ImuFilterBank::default(),
            overflow_sign_change_threshold_rps_squared: 1500.0f32.to_radians() * 1500.0f32.to_radians(),
            gyro_rps_previous: Vector3df32::default(),
            ahrs_data: AhrsData::default(),
            sensor_fusion_filter_is_initializing: true,
            sensor_fusion_filter: MadgwickFilterf32::default(),
        }
    }
}

impl Default for AhrsState {
    fn default() -> Self {
        Self::new()
    }
}

pub trait FuseImuReading<T> {
    fn fuse_acc_gyro_using<F: SensorFusion<T>>(self, filter: &mut F, delta_t: T) -> Quaternion<T>;
}

impl<T> FuseImuReading<T> for ImuReading<T> {
    fn fuse_acc_gyro_using<F: SensorFusion<T>>(self, filter: &mut F, delta_t: T) -> Quaternion<T> {
        let imu_reading = self;
        filter.fuse_acc_gyro(imu_reading.acc, imu_reading.gyro_rps, delta_t)
    }
}

//         //self.ahrs_data.imu_reading.filter_using(self.imu_filters, self.ahrs_data.delta_t);

impl Ahrs for AhrsState {
    fn state(&self) -> &AhrsState {
        self
    }
    fn state_mut(&mut self) -> &mut AhrsState {
        self
    }
    fn read_imu_and_update_orientation(
        &mut self,
        _time_us: u32,
        _imu_filters: ImuFilterBank,
        _vehicle_controller: VehicleController,
    ) -> AhrsData {
        // if the data was read in the IMU interrupt service routine we can just get the data, rather than read it
        //self.ahrs_data.acc_gyro_rps = (_taskType == INTERRUPT_DRIVEN) ? _IMU.get_acc_gyro_rps() : _IMU.read_acc_gyro_rps();

        // Gyros are generally specified to +/- 2000 DPS, in a crash this limit can be exceeded and cause an overflow and a sign reversal in the output.
        self.check_gyro_overflow_z();

        // TODO: this looks ripe for chaining (Ahrs)
        // apply the filters
        self.ahrs_data.gyro_rps_unfiltered = self.ahrs_data.imu_reading.gyro_rps; // unfiltered value saved for blackbox recording
        self.ahrs_data.imu_reading = self.imu_filters.apply(self.ahrs_data.imu_reading, self.ahrs_data.delta_t); // 15us, 207us

        //self.ahrs_data.imu_reading.filter_using(self.imu_filters, self.imu_filters, self.ahrs_data.delta_t);

        /*self.ahrs_data.orientation = self.sensor_fusion_filter.fuse_acc_gyro(
            self.ahrs_data.imu_reading.acc,
            self.ahrs_data.imu_reading.gyro_rps,
            self.ahrs_data.delta_t,
        ); // 15us, 140us*/

        self.ahrs_data.orientation =
            self.ahrs_data.imu_reading.fuse_acc_gyro_using(&mut self.sensor_fusion_filter, self.ahrs_data.delta_t);

        /*if (self.sensor_fusion_filter_is_initializing) {
            if (self.fusion_filter_has_converged(self.ahrs_data.imu_reading.acc, self.ahrs_data.orientation)) {
                self.sensor_fusion_filter_is_initializing = false;
                vehicle_controller.set_sensor_fusion_filter_is_initializing(false);
            }
        }*/

        self.ahrs_data
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn is_normal<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_normal::<AhrsData>();
        is_normal::<AhrsState>();
    }
    #[test]
    fn default_ahrs_data() {
        let ahrs_data = AhrsData::default();
        assert_eq!(0, ahrs_data.time_microseconds);
    }
    #[test]
    fn default_ahrs_state() {
        let ahrs_state = AhrsState::default();
        assert_eq!(true, ahrs_state.sensor_fusion_filter_is_initializing);
    }
    #[test]
    fn new_ahrs_state() {
        let ahrs_state = AhrsState::new();
        assert_eq!(true, ahrs_state.sensor_fusion_filter_is_initializing);
    }
}
