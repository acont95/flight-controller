#include "flight_controller.h"

FlightController::FlightController(ImuManager& imu_manager ,BaroManager& baro_manager, GPSManager& gps_manager, HCRS04Ultrasonic& ultrasonic_sensor, GCSCoordinates home_position) 
    :imu_manager(imu_manager), baro_manager(baro_manager), gps_manager(gps_manager), ultrasonic_sensor(ultrasonic_sensor), home_position(home_position) {
        set_point.location = home_position;
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
        GCSCoordinates loc = gps_manager.getLocation();
        system_state.location.latitude = loc.latitude;
        system_state.location.longitude = loc.longitude;
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
    }
}

void FlightController::readSensors() {
    updateImuData();
    updateBaroData();
    updateGPSData();
    updateUltrasonicData();

    timer.stop();
    setDt(timer.elapsed_time().count());
    timer.reset();
    timer.start();
}

void FlightController::setDt(uint64_t dt) {
    sensor_read_dt = dt;
}

void FlightController::pidUpdate() {

    // Position Hold Outer Loop
    if (set_point.attitude.roll == 0 && set_point.attitude.pitch == 0) {
        error.location.latitude = system_state.location.latitude - set_point.location.latitude;
        error.location.longitude = system_state.location.longitude - set_point.location.longitude;
        set_point.attitude.roll = calculateControlSignal(gains.location_gain, error.location.latitude, previous_error.location.latitude, sensor_read_dt);
        set_point.attitude.pitch = calculateControlSignal(gains.location_gain, error.location.longitude, previous_error.location.longitude, sensor_read_dt);
    } else {
        set_point.location = system_state.location;
    }

    // PID Inner Loop
    uint64_t imu_dt = imu_manager.getDt();

    error.attitude.pitch = system_state.attitude.pitch - set_point.attitude.pitch;
    control_signal.attitude.pitch = calculateControlSignal(gains.pitch_gain, error.attitude.pitch, previous_error.attitude.pitch, imu_dt);
    error.attitude.roll = system_state.attitude.roll - set_point.attitude.roll;
    control_signal.attitude.roll = calculateControlSignal(gains.roll_gain, error.attitude.roll, previous_error.attitude.roll, imu_dt);
    error.attitude.yaw = system_state.attitude.yaw - set_point.attitude.yaw;
    control_signal.attitude.yaw = calculateControlSignal(gains.yaw_gain, error.attitude.yaw, previous_error.attitude.yaw, imu_dt);

    // Alttiude hold
    if (set_point.thrust == 0) {
        error.altitude = system_state.altitude - set_point.altitude;
        control_signal.thrust = calculateControlSignal(gains.thrust_gain, error.altitude, previous_error.altitude, sensor_read_dt);
    } else {
       control_signal.thrust = set_point.thrust;
       set_point.altitude = system_state.altitude;
    }
    

    previous_error = error;
}

int32_t FlightController::calculateControlSignal(Gain gain, int32_t error, int32_t previous_error, uint64_t dt) {
    int32_t proportional = error;
    int32_t integral = (error - previous_error)/2 * dt;
    int32_t derivative = (error - previous_error) / dt;

    return gain.Kp*proportional + gain.Ki*integral + gain.Kd*derivative;
}

void FlightController::testPrint(USBSerial& serial) {

}
