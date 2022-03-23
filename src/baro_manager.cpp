#include "baro_manager.h"

BaroManager::BaroManager(MS5611Barometer& ms5611_barometer) : ms5611_barometer(ms5611_barometer) {
    ms5611_barometer.setOsr(MS5611_OSR_4096);
}

TEMP_PRESSURE_ALTITUDE BaroManager::getAltTempPressure() {
    TEMP_PRESSURE_ALTITUDE res;
    TEMP_AND_PRESSURE sensor_data = ms5611_barometer.getTempAndPressure();
    res.temp = sensor_data.temp;
    res.pressure = sensor_data.pressure;
    res.altitude = (int32_t) (tempPressureToAltitude(sensor_data) * 1000);

    return res;
}

TEMP_AND_PRESSURE BaroManager::getTempAndPressure() {
    return ms5611_barometer.getTempAndPressure();
}

float BaroManager::tempPressureToAltitude(TEMP_AND_PRESSURE sensor_data) {
    return 44330 * (1 - powf((float)sensor_data.pressure/p0, 1/5.225f));
}