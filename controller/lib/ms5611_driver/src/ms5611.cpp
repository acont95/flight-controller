#include "ms5611.hpp"

MS5611Barometer::MS5611Barometer(SPIBusMaster& spi_bus, GPIOOutputInterface& cs_pin, SleepInterface& sleeper) : spi_bus(spi_bus), cs_pin(cs_pin), sleeper(sleeper) {
    // cs_pin.setHigh();
    // spi_bus.format(8, 3);
    // spi_bus.frequency(9000000);
    setOsr(MS5611_OSR_256);
    reset_sleep = 5;
}

void MS5611Barometer::init() {
    reset();
    readProm();
}

void MS5611Barometer::setOsr(MS5611_OSR osr) {
    switch (osr) {
        case MS5611_OSR_256:
            conversion_time = 1;
            d1_register = 0x40;
            d2_register = 0x50;
            break;
        case MS5611_OSR_512:
            conversion_time = 2;
            d1_register = 0x42;
            d2_register = 0x52;
            break;
        case MS5611_OSR_1024:
            conversion_time = 3;
            d1_register = 0x44;
            d2_register = 0x54;
            break;
        case MS5611_OSR_2048:
            conversion_time = 5;
            d1_register = 0x46;
            d2_register = 0x56;
            break;
        case MS5611_OSR_4096:
            conversion_time = 10;
            d1_register = 0x48;
            d2_register = 0x58;
            break;
    }
}

void MS5611Barometer::reset() {
    // readRegister8(MS5611_RESET);
    spi_bus.lock();
    cs_pin.setLow();
    spi_bus.write(MS5611_RESET);
    sleeper.sleepMs(reset_sleep);
    cs_pin.setHigh();
    spi_bus.unlock();

}

void MS5611Barometer::readProm() {
    uint16_t buffer[8];

    for (uint8_t i=0; i<8; i++) {
        buffer[i] = readRegister16(MS5611_PROM_READ + i*2);
    }
    prom.factory_data = buffer[0];
    prom.C1 = buffer[1];
    prom.C2 = buffer[2];
    prom.C3 = buffer[3];
    prom.C4 = buffer[4];
    prom.C5 = buffer[5];
    prom.C6 = buffer[6];
    prom.serial_code_crc = buffer[7];
}

void MS5611Barometer::writeRegister8(uint8_t reg, uint8_t value) {
    spi_bus.lock();
    cs_pin.setLow();
    spi_bus.write(reg);
    spi_bus.write(value);
    cs_pin.setHigh();
    spi_bus.unlock();
}

uint8_t MS5611Barometer::readRegister8(uint8_t reg) {
    spi_bus.lock();
    uint8_t result;
    cs_pin.setLow();
    spi_bus.write(reg);
    result = spi_bus.write(0x00);
    cs_pin.setHigh();
    spi_bus.unlock();

    return result;
}

uint16_t MS5611Barometer::readRegister16(uint8_t reg) {
    uint16_t result;
    spi_bus.lock();
    cs_pin.setLow();
    spi_bus.write(reg);
    uint8_t msb = spi_bus.write(0x00);
    uint8_t lsb = spi_bus.write(0x00);
    cs_pin.setHigh();
    spi_bus.unlock();

    result = (msb << 8) | lsb;

    return result;
}

void MS5611Barometer::writeRegister16(uint8_t reg, uint16_t value) {
    uint8_t msb = value >> 8;
    uint8_t lsb = value & 0xFF;
    spi_bus.lock();
    cs_pin.setLow();
    spi_bus.write(reg);
    spi_bus.write(msb);
    spi_bus.write(lsb);
    cs_pin.setHigh();
    spi_bus.unlock();
}

uint32_t MS5611Barometer::readRegister24(uint8_t reg) {
    uint8_t buf[6] = {0};
    spi_bus.lock();
    cs_pin.setLow();
    spi_bus.write(reg);
    spi_bus.write(buf, 3, buf, 3);
    cs_pin.setHigh();
    spi_bus.unlock();

    return (buf[0] << 16) | (buf[1] << 8) | (buf[2]);
}

uint32_t MS5611Barometer::readUncorrectedPressure() {
    readRegister8(d1_register);
    sleeper.sleepMs(conversion_time);
    return readRegister24(MS5611_ADC_READ);
}

uint32_t MS5611Barometer::readUncorrectedTemp() {
    readRegister8(d2_register);
    sleeper.sleepMs(conversion_time);
    return readRegister24(MS5611_ADC_READ);
}

int32_t MS5611Barometer::getTemperature() {
    uint32_t d2 = readUncorrectedTemp();
    int32_t dt = d2 - prom.C5 * 256;

    return 2000 + dt * prom.C6 / 8388608;
}

int32_t MS5611Barometer::getPressure() {
    uint32_t d1 = readUncorrectedPressure();
    uint32_t d2 = readUncorrectedTemp();
    int32_t dt = d2 - prom.C5 * 256;

    int64_t off = (int64_t)prom.C2 * 65536 + ((int64_t)prom.C4*dt) / 128;
    int64_t sens = (int64_t)prom.C1 * 32768 + ((int64_t)prom.C3*dt) / 256;

    return (d1 * sens / 2097152 - off) / 32768;
}

TEMP_AND_PRESSURE MS5611Barometer::getTempAndPressure() {
    TEMP_AND_PRESSURE res;
    uint32_t d1 = readUncorrectedPressure();
    uint32_t d2 = readUncorrectedTemp();
    int32_t dt = d2 - prom.C5 * 256;

    res.temp = 2000 + dt * prom.C6 / 8388608;

    int64_t off = (int64_t)prom.C2 * 65536 + ((int64_t)prom.C4*dt) / 128;
    int64_t sens = (int64_t)prom.C1 * 32768 + ((int64_t)prom.C3*dt) / 256;

    res.pressure = (d1 * sens / 2097152 - off) / 32768;

    return res;
}

void MS5611Barometer::promToArray(MS5611_PROM prom, uint16_t arr[]) {
    arr[0] = prom.factory_data;
    arr[1] = prom.C1;
    arr[2] = prom.C2;
    arr[3] = prom.C3;
    arr[4] = prom.C4;
    arr[5] = prom.C5;
    arr[6] = prom.C6;
    arr[7] = prom.serial_code_crc;
}

uint8_t MS5611Barometer::crc4(MS5611_PROM prom) {
    uint16_t n_prom[8];
    promToArray(prom, n_prom);

    int cnt; // simple counter
    unsigned int n_rem; // crc reminder
    unsigned int crc_read; // original value of the crc
    unsigned char n_bit;
    n_rem = 0x00;
    crc_read=n_prom[7]; //save read CRC
    n_prom[7]=(0xFF00 & (n_prom[7])); //CRC byte is replaced by 0
    for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
    { // choose LSB or MSB
        if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
            {
            n_rem = (n_rem << 1) ^ 0x3000;
            }
            else
            {
            n_rem = (n_rem << 1);
            }
        }
    }
    n_rem= (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
    n_prom[7]=crc_read; // restore the crc_read to its original place
    return (n_rem ^ 0x00);

}