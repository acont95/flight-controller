#include "icm20948_imu.hpp"

// Constructors
ICM20948_IMU::ICM20948_IMU(SPIBusMaster& spi_bus, GPIOOutputInterface& cs_pin, SleepInterface& sleeper, XYZ16Int accel_offset, XYZ16Int gyro_offset) : spi_bus(spi_bus), cs_pin(cs_pin), sleeper(sleeper), accel_offset(accel_offset), gyro_offset(gyro_offset) {
    // setAccelOffsets();
    // setGyroOffsets();
}

// Public methods

void ICM20948_IMU::setGyroConfig1(GYRO_DLPFCFG gyroDlpf, GYRO_FS_SEL gyroRange, GYRO_FCHOICE gyro_fchoice) {
    uint8_t val = 0;
    val |= (static_cast<int>(gyroDlpf) << 3);
    val |= (static_cast<int>(gyroRange) << 1);
    val |= static_cast<int>(gyro_fchoice);
    writeRegister8(2, ICM20948_GYRO_CONFIG_1, val);
}

void ICM20948_IMU::setUserCtrl(
    DMP_EN dmp_en, 
    FIFO_EN fifo_en,
    I2C_MST_EN i2c_mst_en, 
    I2C_IF_DIS i2c_if_dis, 
    DMP_RST dmp_rst, 
    SRAM_RST sram_rst, 
    I2C_MST_RST i2c_mst_rst
) {
    uint8_t val = 0;
    val |= (static_cast<int>(dmp_en) << 7);
    val |= (static_cast<int>(fifo_en) << 6);
    val |= (static_cast<int>(i2c_mst_en) << 5);
    val |= (static_cast<int>(i2c_if_dis) << 4);
    val |= (static_cast<int>(dmp_rst) << 3);
    val |= (static_cast<int>(sram_rst) << 2);
    val |= (static_cast<int>(i2c_mst_rst) << 1);
    writeRegister8(0, ICM20948_USER_CTRL, val);
 }

void ICM20948_IMU::setLpConfig(I2C_MST_CYCLE i2c_mst_cycle, ACCEL_CYCLE accel_cycle, GYRO_CYCLE gyro_cycle) {
    uint8_t val = 0;
    val |= (static_cast<int>(i2c_mst_cycle) << 6);
    val |= (static_cast<int>(accel_cycle) << 5);
    val |= (static_cast<int>(gyro_cycle) << 4);
    writeRegister8(0, ICM20948_LP_CONFIG, val);
}

void ICM20948_IMU::setPwrMgmt1(DEVICE_RESET device_reset, SLEEP sleep, LP_EN lp_en, TEMP_DIS temp_dis, CLKSEL clk_sel) {
    uint8_t val = 0;
    val |= (static_cast<int>(device_reset) << 7);
    val |= (static_cast<int>(sleep) << 6);
    val |= (static_cast<int>(lp_en) << 5);
    val |= (static_cast<int>(temp_dis) << 3);
    val |= static_cast<int>(clk_sel);
    writeRegister8(0, ICM20948_PWR_MGMT_1, val);
}

void ICM20948_IMU::setPwrMgmt2(GYRO_STATUS gyro_status, ACCEL_STATUS accel_status) {
    uint8_t val = 0;
    val |= (static_cast<int>(accel_status) << 3);
    val |= static_cast<int>(gyro_status);
    writeRegister8(0, ICM20948_PWR_MGMT_2, val);
}

void ICM20948_IMU::setIntPinCfg(
    INT1_ACTL int1_actl, 
    INT1_OPEN int1_open, 
    INT1_LATCH__EN int1_latch_en, 
    INT1_ANYRD_2CLEAR int1_anyrd_2clear, 
    ACTL_FSYNC actl_fsync, 
    FSYNC_INT_MODE_EN fsync_int_mode_en,
    BYPASS_EN bypass_en
) {
    uint8_t val = 0;
    val |= (static_cast<int>(int1_actl) << 7);
    val |= (static_cast<int>(int1_open) << 6);
    val |= (static_cast<int>(int1_latch_en) << 5);
    val |= (static_cast<int>(int1_anyrd_2clear) << 4);
    val |= (static_cast<int>(actl_fsync) << 3);
    val |= (static_cast<int>(fsync_int_mode_en) << 2);
    val |= (static_cast<int>(bypass_en) << 1);

    writeRegister8(0, ICM20948_INT_PIN_CFG, val);
}

void ICM20948_IMU::setIntEnable(
    REG_WOF_EN reg_wof_en, 
    WOM_INT_EN wom_int_en, 
    PLL_RDY_EN pll_rdy_en, 
    DMP_INT1_EN dmp_int1_en, 
    I2C_MST_INT_EN i2c_mst_int_en
) {
    uint8_t val = 0;

    val |= (static_cast<int>(reg_wof_en) << 7);
    val |= (static_cast<int>(wom_int_en) << 3);
    val |= (static_cast<int>(pll_rdy_en) << 2);
    val |= (static_cast<int>(dmp_int1_en) << 1);
    val |= static_cast<int>(i2c_mst_int_en);

    writeRegister8(0, ICM20948_INT_ENABLE, val);
}

void ICM20948_IMU::setIntEnable1(RAW_DATA_0_RDY_EN raw_data_0_rdy_en) {
    writeRegister8(0, ICM20948_INT_ENABLE_1, static_cast<int>(raw_data_0_rdy_en));
}

void ICM20948_IMU::setIntEnable2(FIFO_OVERFLOW_EN fifo_overflow_en) {
    writeRegister8(0, ICM20948_INT_ENABLE_2, static_cast<int>(fifo_overflow_en));
}

void ICM20948_IMU::setIntEnable3(FIFO_WM_EN fifo_wm_en) {
    writeRegister8(0, ICM20948_INT_ENABLE_3, static_cast<int>(fifo_wm_en));
}

I2CMastStatus ICM20948_IMU::getI2cMstStatus() {
    uint8_t val = readRegister8(0, ICM20948_I2C_MST_STATUS);
    I2CMastStatus result;
    
    result.PASS_THROUGH = (val >> 7) & 1;
    result.I2C_SLV4_DONE = (val >> 6) & 1;
    result.I2C_LOST_ARB = (val >> 5) & 1;
    result.I2C_SLV4_NACK = (val >> 4) & 1;
    result.I2C_SLV3_NACK = (val >> 3) & 1;
    result.I2C_SLV2_NACK = (val >> 2) & 1;
    result.I2C_SLV1_NACK = (val >> 1) & 1;
    result.I2C_SLV0_NACK = val & 1;

    return result;
}

IntStatus ICM20948_IMU::getIntStatus() {
    uint8_t val = readRegister8(0, ICM20948_INT_STATUS);
    IntStatus result;

    result.WOM_INT = (val >> 3) & 1;
    result.PLL_RDY_INT = (val >> 2) & 1;
    result.DMP_INT1 = (val >> 1) & 1;
    result.I2C_MST_INT = val & 1;

    return result;
}

IntStatus1 ICM20948_IMU::getIntStatus1() {
    uint8_t val = readRegister8(0, ICM20948_INT_STATUS_1);
    IntStatus1 result;

    result.RAW_DATA_0_RDY_INT = val & 1;
    return result;
}

IntStatus2 ICM20948_IMU::getIntStatus2() {
    uint8_t val = readRegister8(0, ICM20948_INT_STATUS_2);
    IntStatus2 result;
    result.FIFO_OVERFLOW_INT = val & 1;

    return result;
}

IntStatus3 ICM20948_IMU::getIntStatus3() {
    uint8_t val = readRegister8(0, ICM20948_INT_STATUS_3);
    IntStatus3 result;
    result.FIFO_WM_INT = val & 1;

    return result;
}

float ICM20948_IMU::getDelayTime() {
    // delay time in µs
    uint16_t val = readRegister16(0, ICM20948_DELAY_TIMEH);
    return val * 0.9645;
}

float ICM20948_IMU::getTempOut() {
    uint16_t val = readRegister16(0, ICM20948_TEMP_OUT_H);
    return ((val - TEMP_OFFSET) / TEMP_SENSITIVITY) + 21;
}

void ICM20948_IMU::setFifoEn1(
    SLV_3_FIFO_EN slv_3_fifo_en, 
    SLV_2_FIFO_EN slv_2_fifo_en, 
    SLV_1_FIFO_EN slv_1_fifo_en, 
    SLV_0_FIFO_EN slv_0_fifo_en
) {
    uint8_t val = 0;
    val |= (static_cast<int>(slv_3_fifo_en) << 3);
    val |= (static_cast<int>(slv_2_fifo_en) << 2);
    val |= (static_cast<int>(slv_1_fifo_en) << 1);
    val |= static_cast<int>(slv_0_fifo_en);

    writeRegister8(0, ICM20948_FIFO_EN_1, val);    
}


void ICM20948_IMU::setFifoEn2(
    ACCEL_FIFO_EN accel_fifo_en,
    GYRO_Z_FIFO_EN gyro_z_fifo_en,
    GYRO_Y_FIFO_EN gyro_y_fifo_en,
    GYRO_X_FIFO_EN gyro_x_fifo_en,
    TEMP_FIFO_EN temp_fifo_en
) {
    uint8_t val = 0;
    val |= (static_cast<int>(accel_fifo_en) << 4);
    val |= (static_cast<int>(gyro_z_fifo_en) << 3);
    val |= (static_cast<int>(gyro_y_fifo_en) << 2);
    val |= (static_cast<int>(gyro_x_fifo_en) << 1);
    val |= static_cast<int>(temp_fifo_en);

    writeRegister8(0, ICM20948_FIFO_EN_2, val);    
}

void ICM20948_IMU::setFifoRst() {
    writeRegister8(0, ICM20948_FIFO_RST, 1);
    writeRegister8(0, ICM20948_FIFO_RST, 0);
}

void ICM20948_IMU::setFifoMode(FIFO_MODE fifo_mode) {
    writeRegister8(0, ICM20948_FIFO_MODE, static_cast<int>(fifo_mode));
}

uint16_t ICM20948_IMU::getFifoCount() {
    uint16_t val = readRegister16(0, ICM20948_FIFO_COUNT);
    return val;
}


DataRdyStatus ICM20948_IMU::getDataRdyStatus() {
    uint8_t val = readRegister8(0, ICM20948_DATA_RDY_STATUS);
    DataRdyStatus result;
    result.WOF_STATUS = (val >> 7) & 1;
    result.RAW_DATA_RDY = val & 1;

    return result;
}

XYZ16Int ICM20948_IMU::getAccelData() {
    readRegister48(0, ICM20948_ACCEL_XOUT_H);

    int16_t rawX = highLowByteTo16(buf[0], buf[1]);
    int16_t rawY = highLowByteTo16(buf[2], buf[3]);
    int16_t rawZ = highLowByteTo16(buf[4], buf[5]);

    XYZ16Int result;
    result.x = rawX;
    result.y = rawY;
    result.z = rawZ;
    return result;
}

XYZ16Int ICM20948_IMU::getGyroData() {
    readRegister48(0, ICM20948_GYRO_XOUT_H);
    int16_t rawX = highLowByteTo16(buf[0], buf[1]);
    int16_t rawY = highLowByteTo16(buf[2], buf[3]);
    int16_t rawZ = highLowByteTo16(buf[4], buf[5]);

    XYZ16Int result;
    result.x = rawX;
    result.y = rawY;
    result.z = rawZ;
    return result;
}

AccelGyroData ICM20948_IMU::readFifo() {
    readRegister(0, ICM20948_FIFO_R_W, buf, 12);
    AccelGyroData res;
    res.accel.x = highLowByteTo16(buf[0], buf[1]);
    res.accel.y = highLowByteTo16(buf[2], buf[3]);
    res.accel.z = highLowByteTo16(buf[4], buf[5]);
    res.gyro.x = highLowByteTo16(buf[6], buf[7]);
    res.gyro.y = highLowByteTo16(buf[8], buf[9]);
    res.gyro.z = highLowByteTo16(buf[10], buf[11]);

    return res;
}

AccelGyroMagData ICM20948_IMU::readFifoAllSensors() {
    readRegister(0, ICM20948_FIFO_R_W, buf, 21);
    AccelGyroMagData result;
    result.accel.x = highLowByteTo16(buf[0], buf[1]);
    result.accel.y = highLowByteTo16(buf[2], buf[3]);
    result.accel.z = highLowByteTo16(buf[4], buf[5]);
    result.gyro.x = highLowByteTo16(buf[6], buf[7]);
    result.gyro.y = highLowByteTo16(buf[8], buf[9]);
    result.gyro.z = highLowByteTo16(buf[10], buf[11]);
    result.mag_data.mag.x = highLowByteTo16(buf[14], buf[13]);
    result.mag_data.mag.y = highLowByteTo16(buf[16], buf[15]);
    result.mag_data.mag.z = highLowByteTo16(buf[18], buf[17]);

    result.mag_data.dataReady = buf[12] & 0x1;
    result.mag_data.overrun = buf[12] & 0x2;
    result.mag_data.overflow = buf[20] & 0x8;

    return result;
};

uint8_t ICM20948_IMU::getAk09916WhoAmI() {
    return readSingleAK09916Reg(AK09916_WIA2);
}

void ICM20948_IMU::ak09916Init() {
    resetI2cMaster();
    enableI2cMaster();
    // setI2cMstOdrConfig(4);
    setI2cMstCtrl(MULT_MST_EN::DISABLED, I2C_MST_P_NSR::STOP, I2C_MST_CLK::CLK_7);
    setAk09916Control3(AK09916_SRST::RESET);
    setAk09916Control2(AK09916_MODE::CONTINUOUS_MEASUREMENT_MODE4);
    while(getAk09916WhoAmI() != 0x9){};
}

uint8_t ICM20948_IMU::readSingleAK09916Reg(uint8_t reg) {
    setI2cSlv0Addr(I2C_SLV0_RNW::READ, AK09916_SLAVE_ADDRESS);
    setI2cSlv0Reg(reg);
    setI2cSlv0Ctrl(I2C_SLV0_EN::ENABLED, I2C_SLV0_BYTE_SW::NO_SWAPPING, I2C_SLV0_REG_DIS::DISABLED, I2C_SLV0_GRP::ODD_NUMBERING, 1);
    sleeper.sleepMs(1);
    uint8_t result = readRegister8(0, ICM20948_EXT_SLV_SENS_DATA_00);

    return result;
}

void ICM20948_IMU::readAk09916Reg(uint8_t reg, uint8_t len) {
    setI2cSlv0Addr(I2C_SLV0_RNW::READ, AK09916_SLAVE_ADDRESS);
    setI2cSlv0Reg(reg);
    setI2cSlv0Ctrl(I2C_SLV0_EN::ENABLED, I2C_SLV0_BYTE_SW::NO_SWAPPING, I2C_SLV0_REG_DIS::DISABLED, I2C_SLV0_GRP::ODD_NUMBERING, len);
}

AK09916Data ICM20948_IMU::readAk09916ExtData() {
    AK09916Data result;
    uint8_t dummy[9] = {0};
    readRegister(0, ICM20948_EXT_SLV_SENS_DATA_00, dummy, 9);
    result.mag.x = highLowByteTo16(buf[2], buf[1]);
    result.mag.y = highLowByteTo16(buf[4], buf[3]);
    result.mag.z = highLowByteTo16(buf[6], buf[5]);
    result.dataReady = buf[0] & 0x1;
    result.overrun = buf[0] & 0x2;
    result.overflow = buf[8] & 0x8;

    return result;
}

void ICM20948_IMU::writeSingleAK09916Reg(uint8_t reg, uint8_t val) {
    setI2cSlv0Addr(I2C_SLV0_RNW::WRITE, AK09916_SLAVE_ADDRESS);
    setI2cSlv0Reg(reg);
    setI2cSlv0Do(val);
    setI2cSlv0Ctrl(I2C_SLV0_EN::ENABLED, I2C_SLV0_BYTE_SW::NO_SWAPPING, I2C_SLV0_REG_DIS::DISABLED, I2C_SLV0_GRP::ODD_NUMBERING, 1);
}

void ICM20948_IMU::resetI2cMaster() {
    uint8_t current = readRegister8(0, ICM20948_USER_CTRL);
    current |= 0x2;
    writeRegister8(0, ICM20948_USER_CTRL, current);
}

void ICM20948_IMU::enableI2cMaster() {
    uint8_t current = readRegister8(0, ICM20948_USER_CTRL);
    current |= 0x20;
    writeRegister8(0, ICM20948_USER_CTRL, current);
    sleeper.sleepMs(100);
}


void ICM20948_IMU::setAccelOffsets() {
    XYZ16Int factoryAccelOffsets = getAccelOffsets();
    uint8_t mask[3] = {0,0,0};

    if (factoryAccelOffsets.x & 0x1) {
        mask[0] = 0x1;
    }

    if (factoryAccelOffsets.y & 0x1) {
        mask[1] = 0x1;
    }

    if (factoryAccelOffsets.z & 0x1) {
        mask[2] = 0x1;
    }

    buf[0] = (factoryAccelOffsets.x - accel_offset.x) >> 8;
    buf[1] = (factoryAccelOffsets.x - accel_offset.x) & 0xFF;
    buf[1] = buf[1] | mask[0];
    buf[2] = (factoryAccelOffsets.y - accel_offset.y) >> 8;
    buf[3] = (factoryAccelOffsets.y - accel_offset.y) & 0xFF;
    buf[3] = buf[3] | mask[1];
    buf[4] = (factoryAccelOffsets.z - accel_offset.z) >> 8;
    buf[5] = (factoryAccelOffsets.z - accel_offset.z) & 0xFF;
    buf[5] = buf[5] | mask[2];

    readRegister48(1, ICM20948_XA_OFFS_H);
}

XYZ16Int ICM20948_IMU::getAccelOffsets() {
    XYZ16Int result;
    readRegister48(1, ICM20948_XA_OFFS_H);
    result.x = highLowByteTo16(buf[0], buf[1]);
    result.y = highLowByteTo16(buf[2], buf[3]);
    result.z = highLowByteTo16(buf[4], buf[5]);
    return result;
}

float ICM20948_IMU::getTimeBaseCorrectionPll() {
    float step = 0.079f;
    int8_t val = (int8_t) readRegister8(1, ICM20948_TIMEBASE_CORRECTION_PLL);
    return val * step;
}

void ICM20948_IMU::setGyroSmplrtDiv(uint8_t smplrt) {
    writeRegister8(2, ICM20948_GYRO_SMLRT_DIV, smplrt);
}

void ICM20948_IMU::setGyroConfig2(XGYRO_CTEN x_gyro_cten, YGYRO_CTEN y_gyro_cten, ZGYRO_CTEN z_gyro_cten, GYRO_AVGCFG gyro_avgcf) {
    uint8_t val = 0;
    val |= (static_cast<int>(x_gyro_cten) << 5);
    val |= (static_cast<int>(y_gyro_cten) << 4);
    val |= (static_cast<int>(z_gyro_cten) << 3);
    val |= static_cast<int>(gyro_avgcf);

    writeRegister8(2, ICM20948_GYRO_CONFIG_2, val);
}

void ICM20948_IMU::setGyroOffsets() {
    writeRegister8(2, ICM20948_XG_OFFS_USRH, (uint8_t)(gyro_offset.x >> 8));
    writeRegister8(2, ICM20948_XG_OFFS_USRL, (uint8_t)(gyro_offset.x & 0xFF));
    writeRegister8(2, ICM20948_YG_OFFS_USRH, (uint8_t)(gyro_offset.y >> 8));
    writeRegister8(2, ICM20948_YG_OFFS_USRL, (uint8_t)(gyro_offset.y & 0xFF));
    writeRegister8(2, ICM20948_ZG_OFFS_USRH, (uint8_t)(gyro_offset.z >> 8));
    writeRegister8(2, ICM20948_ZG_OFFS_USRL, (uint8_t)(gyro_offset.z & 0xFF));
}

void ICM20948_IMU::setOdrAlignEn(ODR_ALIGN_EN odr_align_en) {
    writeRegister8(2, ICM20948_ODR_ALIGN_EN, static_cast<int>(odr_align_en));
}

void ICM20948_IMU::setAccelSmplrtDiv(uint16_t smplrt) {
    writeRegister16(2, ICM20948_ACCEL_SMPLRT_DIV_1, smplrt);
}

void ICM20948_IMU::setAccelIntelCtrl(ACCEL_INTEL_EN accel_intel_en, ACCEL_INTEL_MODE_INT accel_intel_mode_int) {
    uint8_t val = 0;
    val |= (static_cast<int>(accel_intel_en) << 1);
    val |= static_cast<int>(accel_intel_mode_int);

    writeRegister8(2, ICM20948_ACCEL_INTEL_CTRL, val);
}

void ICM20948_IMU::setAccelWomThr(uint8_t wom_thr) {
    writeRegister8(2, ICM20948_ACCEL_WOM_THR, wom_thr);
}

void ICM20948_IMU::setAccelConfig(ACCEL_DLPFCFG accel_dlpfcfg, ACCEL_FS_SEL accel_fs_sel, ACCEL_FCHOICE accel_fchoice) {
    uint8_t val = 0;
    val |= (static_cast<int>(accel_dlpfcfg) << 3);
    val |= (static_cast<int>(accel_fs_sel) << 1);
    val |= (static_cast<int>(accel_fchoice));

    writeRegister8(2, ICM20948_ACCEL_CONFIG, val);
}

void ICM20948_IMU::setAccelConfig2(AX_ST_EN_REG ax_st_en_reg, AY_ST_EN_REG ay_st_en_reg, AZ_ST_EN_REG az_st_en_reg, DEC3_CFG dec3_cfg) {
    uint8_t val = 0;
    val |= (static_cast<int>(ax_st_en_reg) << 4);
    val |= (static_cast<int>(ay_st_en_reg) << 3);
    val |= (static_cast<int>(az_st_en_reg) << 2);
    val |= static_cast<int>(dec3_cfg);

    writeRegister8(2, ICM20948_ACCEL_CONFIG_2, val);
}

void ICM20948_IMU::setFsyncConfig(DELAY_TIME_EN delay_time_en, WOF_DEGLITCH_EN wof_deglitch_en, WOF_EDGE_INT wof_edge_int, EXT_SYNC_SET ext_sync_set) {
    uint8_t val = 0;
    val |= (static_cast<int>(delay_time_en) << 7);
    val |= (static_cast<int>(wof_deglitch_en) << 5);
    val |= (static_cast<int>(wof_edge_int) << 4);
    val |= static_cast<int>(ext_sync_set);

    writeRegister8(2, ICM20948_FSYNC_CONFIG, val);
}

void ICM20948_IMU::setTempConfig(TEMP_DLPFCFG temp_dlpfcfg) {
    uint8_t val = 0;
    val |= static_cast<int>(temp_dlpfcfg);

    writeRegister8(2, ICM20948_TEMP_CONFIG, val);
}

void ICM20948_IMU::setModCtrlUsr(REG_LP_DMP_EN reg_lp_dmp_en) {
    uint8_t val = 0;
    val |= static_cast<int>(reg_lp_dmp_en);

    writeRegister8(2, ICM20948_MOD_CTRL_USR, val);
}

AK09916Status1 ICM20948_IMU::getAk09916Status1() {
    AK09916Status1 result;
    uint8_t val = readRegister8(0, AK09916_ST1);
    result.DRDY = val & 1;
    result.DOR = (val >> 1) & 1;

    return result;
}

void ICM20948_IMU::setupAk09916MeasurementData() {
    readAk09916Reg(AK09916_ST1, 9);
}

AK09916Status2 ICM20948_IMU::getAk09916Status2() {
    AK09916Status2 result;
    uint8_t val = readRegister8(0, AK09916_ST2);
    result.HOFL = (val >> 3) & 1;

    return result;
}

void ICM20948_IMU::setAk09916Control2(AK09916_MODE mode) {
    writeSingleAK09916Reg(AK09916_CNTL2, static_cast<int>(mode));
    sleeper.sleepMs(100);
}

void ICM20948_IMU::setAk09916Control3(AK09916_SRST srst) {
    writeSingleAK09916Reg(AK09916_CNTL3, static_cast<int>(srst));
    sleeper.sleepMs(100);
}

float ICM20948_IMU::getGyroScaleFactor(GYRO_FS_SEL gyro_full_scale) {
    switch (gyro_full_scale)
    {
    case GYRO_FS_SEL::GYRO_RANGE_250 :
        return 131.0f;

    case GYRO_FS_SEL::GYRO_RANGE_500 :
        return 65.5f;
    
    case GYRO_FS_SEL::GYRO_RANGE_1000 :
        return 32.8f;

    case GYRO_FS_SEL::GYRO_RANGE_2000 :
        return 16.4f;
    }
}

float ICM20948_IMU::getAccelScaleFactor(ACCEL_FS_SEL accel_full_scale) {
    switch (accel_full_scale)
    {
    case ACCEL_FS_SEL::ACCEL_FULL_SCALE_2G :
        return 16384.0f;

    case ACCEL_FS_SEL::ACCEL_FULL_SCALE_4G :
        return 8192.0f;
    
    case ACCEL_FS_SEL::ACCEL_FULL_SCALE_8G :
        return 4096.0f;

    case ACCEL_FS_SEL::ACCEL_FULL_SCALE_16G :
        return 2048.0f;
    }
}

GyroConfig1 ICM20948_IMU::getGyroConfig1() {
    GyroConfig1 result;
    uint8_t val = readRegister8(2, ICM20948_GYRO_CONFIG_1);
    result.gyro_dlpfcfg = static_cast<GYRO_DLPFCFG>((val >> 3) & 0x7);
    result.gyro_fs_sel = static_cast<GYRO_FS_SEL>((val >> 1) & 0x3);
    result.gyro_fchoice = static_cast<GYRO_FCHOICE>(val & 0x1);
    return result;
}

AccelConfig ICM20948_IMU::getAccelConfig() {
    AccelConfig result;
    uint8_t val = readRegister8(2, ICM20948_ACCEL_CONFIG);
    result.accel_dlpfcfg = static_cast<ACCEL_DLPFCFG>((val >> 3) & 0x7);
    result.accel_fs_sel = static_cast<ACCEL_FS_SEL>((val >> 1) & 0x3);
    result.accel_fchoice = static_cast<ACCEL_FCHOICE>(val & 0x1);
    return result;
}
 
// Private methods

void ICM20948_IMU::readRegister48(uint8_t bank, uint8_t reg) {
    reg |= 0x80;
    switchBank(bank);
    spi_bus.lock();
    cs_pin.setLow();
    spi_bus.write(reg);
    spi_bus.write(buf, 6, buf, 6);
    cs_pin.setHigh();
    spi_bus.unlock();
}

void ICM20948_IMU::writeRegister8(uint8_t bank, uint8_t reg, uint8_t value) {
    switchBank(bank);
    spi_bus.lock();
    cs_pin.setLow();
    spi_bus.write(reg);
    spi_bus.write(value);
    cs_pin.setHigh();
    spi_bus.unlock();
}

uint8_t ICM20948_IMU::readRegister8(uint8_t bank, uint8_t reg) {
    reg |= 0x80;
    uint8_t result = 0;
    switchBank(bank);
    spi_bus.lock();
    cs_pin.setLow();
    spi_bus.write(reg);
    result = spi_bus.write(0x00);
    cs_pin.setHigh();
    spi_bus.unlock();

    return result;
}

int16_t ICM20948_IMU::readRegister16(uint8_t bank, uint8_t reg) {
    reg |= 0x80;
    int16_t result;
    switchBank(bank);
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

void ICM20948_IMU::writeRegister16(uint8_t bank, uint8_t reg, uint16_t value) {
    uint8_t msb = value >> 8;
    uint8_t lsb = value & 0xFF;
    switchBank(bank);
    spi_bus.lock();
    cs_pin.setLow();
    spi_bus.write(reg);
    spi_bus.write(msb);
    spi_bus.write(lsb);
    cs_pin.setHigh();
    spi_bus.unlock();
}

void ICM20948_IMU::switchBank(uint8_t bank) {
    if (bank != currentBank) {
        currentBank = bank;
        spi_bus.lock();
        cs_pin.setLow();
        spi_bus.write(ICM20948_REG_BANK_SEL);
        spi_bus.write(bank<<4);
        cs_pin.setHigh();
        spi_bus.unlock();
    }
}

void ICM20948_IMU::writeRegister(uint8_t bank, uint8_t reg, uint8_t data[], uint8_t size) {
    switchBank(bank);
    spi_bus.lock();
    cs_pin.setLow();
    spi_bus.write(reg);
    spi_bus.write(data, size, data, size);
    cs_pin.setHigh();
    spi_bus.unlock();
}

void ICM20948_IMU::readRegister(uint8_t bank, uint8_t reg, uint8_t data[], uint8_t size) {
    uint8_t dummy_data[size] = {0}; 
    reg |= 0x80;
    switchBank(bank);
    spi_bus.lock();
    cs_pin.setLow();
    spi_bus.write(reg);
    spi_bus.write(dummy_data, size, buf, size);
    cs_pin.setHigh();
    spi_bus.unlock();
}

void ICM20948_IMU::setI2cMstOdrConfig(uint8_t smplrt) {
    uint8_t val = 0;
    val |= (static_cast<int>(smplrt));

    writeRegister8(3, ICM20948_I2C_MST_ODR_CONFIG, val);
}

void ICM20948_IMU::setI2cMstCtrl(MULT_MST_EN mult_mst_en, I2C_MST_P_NSR i2c_mst_p_nsr, I2C_MST_CLK i2c_mst_clk) {
    uint8_t val = 0;
    val |= (static_cast<int>(mult_mst_en) << 7);
    val |= (static_cast<int>(i2c_mst_p_nsr) << 4);
    val |= (static_cast<int>(i2c_mst_clk));

    writeRegister8(3, ICM20948_I2C_MST_CTRL, val);
}

void ICM20948_IMU::setI2cMstDelayCtrl(DELAY_ES_SHADOW delay_es_shadow, I2C_SLV4_DELAY_EN i2c_slv4_delay_en, I2C_SLV3_DELAY_EN i2c_slv3_delay_en,
    I2C_SLV2_DELAY_EN i2c_slv2_delay_en, I2C_SLV1_DELAY_EN i2c_slv1_delay_en, I2C_SLV0_DELAY_EN i2c_slv0_delay_en) {
        uint8_t val = 0;
        val |= (static_cast<int>(delay_es_shadow) << 7);
        val |= (static_cast<int>(i2c_slv4_delay_en) << 4);
        val |= (static_cast<int>(i2c_slv3_delay_en) << 3);
        val |= (static_cast<int>(i2c_slv2_delay_en) << 2);
        val |= (static_cast<int>(i2c_slv1_delay_en) << 1);
        val |= (static_cast<int>(i2c_slv0_delay_en));

        writeRegister8(3, ICM20948_I2C_MST_DELAY_CTRL, val);
    }

void ICM20948_IMU::setI2cSlv0Addr(I2C_SLV0_RNW i2c_slv0_rnw, uint8_t i2c_id_0) {
    uint8_t val = 0;
    val |= (static_cast<int>(i2c_slv0_rnw) << 7);
    val |= (i2c_id_0 & 0x7F);

    writeRegister8(3, ICM20948_I2C_SLV0_ADDR, val);
}

void ICM20948_IMU::setI2cSlv0Reg(uint8_t reg) {
    writeRegister8(3, ICM20948_I2C_SLV0_REG, reg);
}

void ICM20948_IMU::setI2cSlv0Ctrl(I2C_SLV0_EN i2c_slv0_en, I2C_SLV0_BYTE_SW i2c_slv0_byte_sw, I2C_SLV0_REG_DIS i2c_slv0_reg_dis, I2C_SLV0_GRP i2c_slv0_grp, uint8_t i2c_slvo_leng) {
    uint8_t val = 0;
    val |= (static_cast<int>(i2c_slv0_en) << 7);
    val |= (static_cast<int>(i2c_slv0_byte_sw) << 6);
    val |= (static_cast<int>(i2c_slv0_reg_dis) << 5);
    val |= (static_cast<int>(i2c_slv0_grp) << 4);
    val |= (i2c_slvo_leng & 0xF);
    writeRegister8(3, ICM20948_I2C_SLV0_CTRL, val);
}

void ICM20948_IMU::setI2cSlv0Do(uint8_t data) {
    writeRegister8(3, ICM20948_I2C_SLV0_DO, data);
}

void ICM20948_IMU::switchDmpBank(uint8_t bank) {
    if (bank != currentDmpBank) {
        switchBank(0);
        currentDmpBank = bank;
        spi_bus.lock();
        cs_pin.setLow();
        spi_bus.write(DMP_MEM_BANK_SEL);
        spi_bus.write(bank);
        cs_pin.setHigh();
        spi_bus.unlock();
    }
}

void ICM20948_IMU::loadDmpFirmware(uint8_t dmp3_image[], uint16_t dmp3_image_size, uint8_t load_addr) {
    uint8_t* data = dmp3_image;
    uint16_t size = dmp3_image_size;
    uint8_t mem_addr = load_addr;
    // uint8_t data_cmp[INV_MAX_SERIAL_READ];

    int write_size;
    while (size > 0) {
        write_size = MIN(size, INV_MAX_SERIAL_WRITE);
        if ((mem_addr & 0xff) + write_size > 0x100) {
            // Moved across a bank
            write_size = (mem_addr & 0xff) + write_size - 0x100;
        }
        switchDmpBank(mem_addr >> 8);
        writeRegister8(0, DMP_MEM_START_ADDR, (mem_addr & 0xff));
        writeRegister(0, DMP_MEM_R_W, data, write_size);

        data += write_size;
        size -= write_size;
        mem_addr += write_size;
    }

    data = dmp3_image;
    size = dmp3_image_size;
    mem_addr = load_addr;
    // uint8_t data_cmp[INV_MAX_SERIAL_WRITE];

    while (size > 0) {
        write_size = MIN(size, INV_MAX_SERIAL_WRITE);
        if ((mem_addr & 0xff) + write_size > 0x100) {
            // Moved across a bank
            write_size = (mem_addr & 0xff) + write_size - 0x100;
        }
    }

}

void ICM20948_IMU::setDmpStartAddress() {
    writeRegister16(2, PRGM_STRT_ADDRH, DMP_START_ADDRESS);
}

int16_t ICM20948_IMU::highLowByteTo16(uint8_t highByte, uint8_t lowByte) {
    return (int16_t) ((highByte << 8) | lowByte);
}

const float ICM20948_IMU::TEMP_SENSITIVITY = 333.87f;
const float ICM20948_IMU::TEMP_OFFSET = 0.0f;