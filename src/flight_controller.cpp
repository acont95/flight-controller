#include "flight_controller.h"

FlightController::FlightController(ImuManager& imu_manager ,BaroManager& baro_manager, GPSManager& gps_manager, HCRS04Ultrasonic& ultrasonic_sensor) 
    :imu_manager(imu_manager), baro_manager(baro_manager), gps_manager(gps_manager), ultrasonic_sensor(ultrasonic_sensor) {
}

void FlightController::updateImuAttitude(Attitude attitude) {
    system_state.attitude = attitude;
}

void FlightController::updateImuData() {
    Attitude attitude = imu_manager.getAttitude();
    system_state.attitude.roll = attitude.roll;
    system_state.attitude.pitch = attitude.pitch;
    system_state.attitude.yaw = attitude.yaw;
}

void FlightController::updateBaroData() {
    TEMP_PRESSURE_ALTITUDE temp_pressure_attitude = baro_manager.getAltTempPressure();
    baro_data = temp_pressure_attitude;
    system_state.altitude = temp_pressure_attitude.altitude;
}

void FlightController::updateGPSData() {
    if (gps_manager.getGPS().location.isUpdated()) {
        system_state.location.latitude = gps_manager.getGPS().location.rawLat();
        system_state.location.longitude = gps_manager.getGPS().location.rawLng();
    }
    if (gps_manager.getGPS().altitude.isUpdated()) {
        system_state.altitude = gps_manager.getGPS().altitude.value();
    }
}

void FlightController::updateUltrasonicData() {
    if (ultrasonic_sensor.shouldTrigger()) {
        ultrasonic_sensor.triggerPulse();
    } 
    if (ultrasonic_sensor.readReady()) {
        uint16_t d_mm = ultrasonic_sensor.getDistanceMm();
        system_state.height_above_ground = d_mm;
        // serial.printf("Distance: %i\n", (int)d_mm);
    }
}

void FlightController::readSensors() {
    updateImuData();
    updateBaroData();
    updateGPSData();
    updateUltrasonicData();
}

void FlightController::pidUpdate() {
    error.attitude.pitch = system_state.attitude.pitch - set_point.attitude.pitch;
    error.attitude.roll = system_state.attitude.roll - set_point.attitude.roll;
    error.attitude.yaw = system_state.attitude.yaw - set_point.attitude.roll;

    int32_t pitch_proportional_error = error.attitude.pitch;
    int32_t pitch_integral_error = (error.attitude.pitch - previous_error.attitude.pitch)/2 * pidUpdatePeriod.count();
    int32_t pitch_derivative_error = (error.attitude.pitch - previous_error.attitude.pitch) / pidUpdatePeriod.count();
    control_signal.attitude.pitch = gains.pitch_gain.Kp*pitch_proportional_error + gains.pitch_gain.Ki*pitch_integral_error + gains.pitch_gain.Kd*pitch_derivative_error;

    int32_t roll_proportional_error = error.attitude.roll;
    int32_t roll_integral_error = (error.attitude.roll - previous_error.attitude.roll)/2 * pidUpdatePeriod.count();
    int32_t roll_derivative_error = (error.attitude.roll - previous_error.attitude.roll) / pidUpdatePeriod.count();
    control_signal.attitude.roll = gains.roll_gain.Kp*roll_proportional_error + gains.roll_gain.Ki*roll_integral_error + gains.roll_gain.Kd*roll_derivative_error;

    int32_t yaw_proportional_error = error.attitude.yaw;
    int32_t yaw_integral_error = (error.attitude.yaw - previous_error.attitude.yaw)/2 * pidUpdatePeriod.count();
    int32_t yaw_derivative_error = (error.attitude.yaw - previous_error.attitude.yaw) / pidUpdatePeriod.count();
    control_signal.attitude.yaw = gains.yaw_gain.Kp*yaw_proportional_error + gains.yaw_gain.Ki*yaw_integral_error + gains.yaw_gain.Kd*yaw_derivative_error;
}


