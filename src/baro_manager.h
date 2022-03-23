#pragma once

#include <ms5611.h>
#include <cmath>
// #include <libfixmath/fixmath.h>
// #undef abs
// #include <fpm/fixed.hpp>
// #include <fpm/math.hpp>

typedef struct TEMP_PRESSURE_ALTITUDE {
    int32_t temp;
    int32_t pressure;
    int32_t altitude;
} TEMP_PRESSURE_ALTITUDE;

class BaroManager {
    // Reference: https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5611-01BA03%7FB3%7Fpdf%7FEnglish%7FENG_DS_MS5611-01BA03_B3.pdf%7FCAT-BLPS0036
    // Reference: https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
    public:
        BaroManager(MS5611Barometer& ms5611_barometer);
        TEMP_PRESSURE_ALTITUDE getAltTempPressure();
        TEMP_AND_PRESSURE getTempAndPressure();
        float tempPressureToAltitude(TEMP_AND_PRESSURE sensor_data);

    private:
        MS5611Barometer& ms5611_barometer;
        static const uint32_t p0 = 101325;
};