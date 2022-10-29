#pragma once

#include <cstdint>
// #include <mbed.h>


#define MS5611_RESET 0x1E
#define MS5611_CONVERT_D1_256 0x40
#define MS5611_CONVERT_D1_512 0x42
#define MS5611_CONVERT_D1_1024 0x44
#define MS5611_CONVERT_D1_2048 0x46
#define MS5611_CONVERT_D1_4096 0x48
#define MS5611_CONVERT_D2_256 0x50
#define MS5611_CONVERT_D2_512 0x52
#define MS5611_CONVERT_D2_1024 0x54
#define MS5611_CONVERT_D2_2048 0x56
#define MS5611_CONVERT_D2_4096 0x58
#define MS5611_ADC_READ 0x00
#define MS5611_PROM_READ 0xA0

typedef struct MS5611_PROM {
    uint16_t factory_data;
    uint16_t C1;
    uint16_t C2;
    uint16_t C3;
    uint16_t C4;
    uint16_t C5;
    uint16_t C6;
    uint16_t serial_code_crc;

} MS5611_PROM;

typedef struct TEMP_AND_PRESSURE {
    int32_t temp;
    int32_t pressure;
} TEMP_AND_PRESSURE;

typedef enum MS5611_OSR {
    MS5611_OSR_256,
    MS5611_OSR_512,
    MS5611_OSR_1024,
    MS5611_OSR_2048,
    MS5611_OSR_4096
} MS561_OSR;


class MS5611Barometer {
    public:
        MS5611Barometer(mbed::SPI& spi_bus, mbed::DigitalOut& cs_pin);
        void reset();
        void readProm();
        int32_t getTemperature();
        int32_t getPressure();
        TEMP_AND_PRESSURE getTempAndPressure();
        void setOsr(MS5611_OSR osr);
        MS5611_PROM prom;
        uint8_t crc4(MS5611_PROM prom);


    private:
        mbed::SPI& spi_bus;
        mbed::DigitalOut& cs_pin;
        void writeRegister8(uint8_t reg, uint8_t value);
        uint8_t readRegister8(uint8_t reg);
        void writeRegister16(uint8_t reg, uint16_t value);
        uint16_t readRegister16(uint8_t reg);
        uint32_t readRegister24(uint8_t reg);
        uint32_t readUncorrectedPressure();
        uint32_t readUncorrectedTemp();
        uint8_t d1_register;
        uint8_t d2_register;
        void promToArray(MS5611_PROM prom, uint16_t arr[]); 

        rtos::Kernel::Clock::duration_u32 reset_sleep;
        rtos::Kernel::Clock::duration_u32 conversion_time;
};