#pragma once

#include <cstdint>
#include <mbed.h>

// USER BANK 0 REGISTER MAP
#define ICM20948_WHO_AM_I 0x00
#define ICM20948_USER_CTRL 0x03
#define ICM20948_LP_CONFIG 0x05
#define ICM20948_PWR_MGMT_1 0x06
#define ICM20948_PWR_MGMT_2 0x07
#define ICM20948_INT_PIN_CFG 0x0F
#define ICM20948_INT_ENABLE 0x10
#define ICM20948_INT_ENABLE_1 0x11
#define ICM20948_INT_ENABLE_2 0x12
#define ICM20948_INT_ENABLE_3 0x13
#define ICM20948_I2C_MST_STATUS 0x17
#define ICM20948_INT_STATUS 0x19
#define ICM20948_INT_STATUS_1 0x1A
#define ICM20948_INT_STATUS_2 0x1B
#define ICM20948_INT_STATUS_3 0x1C
#define ICM20948_DELAY_TIMEH 0x28
#define ICM20948_DELAY_TIMEL 0x29
#define ICM20948_ACCEL_XOUT_H 0x2D
#define ICM20948_ACCEL_XOUT_L 0x2E
#define ICM20948_ACCEL_YOUT_H 0x2F
#define ICM20948_ACCEL_YOUT_L 0x30
#define ICM20948_ACCEL_ZOUT_H 0x31
#define ICM20948_ACCEL_ZOUT_L 0x32
#define ICM20948_GYRO_XOUT_H 0x33
#define ICM20948_GYRO_XOUT_L 0x34
#define ICM20948_GYRO_YOUT_H 0x35
#define ICM20948_GYRO_YOUT_L 0x36
#define ICM20948_GYRO_ZOUT_H 0x37
#define ICM20948_GYRO_ZOUT_L 0x38
#define ICM20948_TEMP_OUT_H 0x39
#define ICM20948_TEMP_OUT_L 0x3A
#define ICM20948_EXT_SLV_SENS_DATA_00 0x3B
#define ICM20948_EXT_SLV_SENS_DATA_01 0x3C
#define ICM20948_EXT_SLV_SENS_DATA_02 0x3D
#define ICM20948_EXT_SLV_SENS_DATA_03 0x3E
#define ICM20948_EXT_SLV_SENS_DATA_04 0x3F
#define ICM20948_EXT_SLV_SENS_DATA_05 0x40
#define ICM20948_EXT_SLV_SENS_DATA_06 0x41
#define ICM20948_EXT_SLV_SENS_DATA_07 0x42
#define ICM20948_EXT_SLV_SENS_DATA_08 0x43
#define ICM20948_EXT_SLV_SENS_DATA_09 0x44
#define ICM20948_EXT_SLV_SENS_DATA_10 0x45
#define ICM20948_EXT_SLV_SENS_DATA_11 0x46
#define ICM20948_EXT_SLV_SENS_DATA_12 0x47
#define ICM20948_EXT_SLV_SENS_DATA_13 0x48
#define ICM20948_EXT_SLV_SENS_DATA_14 0x49
#define ICM20948_EXT_SLV_SENS_DATA_15 0x4A
#define ICM20948_EXT_SLV_SENS_DATA_16 0x4B
#define ICM20948_EXT_SLV_SENS_DATA_17 0x4C
#define ICM20948_EXT_SLV_SENS_DATA_18 0x4D
#define ICM20948_EXT_SLV_SENS_DATA_19 0x4E
#define ICM20948_EXT_SLV_SENS_DATA_20 0x4F
#define ICM20948_EXT_SLV_SENS_DATA_21 0x50
#define ICM20948_EXT_SLV_SENS_DATA_22 0x51
#define ICM20948_EXT_SLV_SENS_DATA_23 0X52
#define ICM20948_FIFO_EN_1 0x66
#define ICM20948_FIFO_EN_2 0x67
#define ICM20948_FIFO_RST 0x68
#define ICM20948_FIFO_MODE 0x69
#define ICM20948_FIFO_COUNT 0x70
#define ICM20948_FIFO_R_W 0x72
#define ICM20948_DATA_RDY_STATUS 0x74
#define ICM20948_FIFO_CFG 0x76
#define ICM20948_REG_BANK_SEL 0x7F

// USER BANK 1 REGISTER MAP
#define ICM20948_SELF_TEST_X_GYRO 0x02
#define ICM20948_SELF_TEST_Y_GYRO 0x03
#define ICM20948_SELF_TEST_Z_GYRO 0x04
#define ICM20948_SELF_TEST_X_ACCEL 0x0E
#define ICM20948_SELF_TEST_Y_ACCEL 0x0F
#define ICM20948_SELF_TEST_Z_ACCEL 0x10
#define ICM20948_XA_OFFS_H 0x14
#define ICM20948_XA_OFFS_L 0x15
#define ICM20948_YA_OFFS_H 0x17
#define ICM20948_YA_OFFS_L 0x18
#define ICM20948_ZA_OFFS_H 0x1A
#define ICM20948_ZA_OFFS_L 0x1B
#define ICM20948_TIMEBASE_CORRECTION_PLL 0x28

// USER BANK 2 REGISTER MAP
#define ICM20948_GYRO_SMLRT_DIV 0x00
#define ICM20948_GYRO_CONFIG_1 0x01
#define ICM20948_GYRO_CONFIG_2 0x02
#define ICM20948_XG_OFFS_USRH 0x03
#define ICM20948_XG_OFFS_USRL 0x04
#define ICM20948_YG_OFFS_USRH 0x05
#define ICM20948_YG_OFFS_USRL 0x05
#define ICM20948_ZG_OFFS_USRH 0x07
#define ICM20948_ZG_OFFS_USRL 0x08
#define ICM20948_ODR_ALIGN_EN 0x09
#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_ACCEL_INTEL_CTRL 0x12
#define ICM20948_ACCEL_WOM_THR 0x13
#define ICM20948_ACCEL_CONFIG 0x14
#define ICM20948_ACCEL_CONFIG_2 0x15
#define ICM20948_FSYNC_CONFIG 0x52
#define ICM20948_TEMP_CONFIG 0x53
#define ICM20948_MOD_CTRL_USR 0x54

// USER BANK 3 REGISTER MAP
#define ICM20948_I2C_MST_ODR_CONFIG 0x00
#define ICM20948_I2C_MST_CTRL 0x01
#define ICM20948_I2C_MST_DELAY_CTRL 0x02
#define ICM20948_I2C_SLV0_ADDR 0x03
#define ICM20948_I2C_SLV0_REG 0x04
#define ICM20948_I2C_SLV0_CTRL 0x05
#define ICM20948_I2C_SLV0_DO 0x06
#define ICM20948_I2C_SLV1_ADDR 0x07
#define ICM20948_I2C_SLV1_REG 0x08
#define ICM20948_I2C_SLV1_CTRL 0x09
#define ICM20948_I2C_SLV1_DO 0x0A
#define ICM20948_I2C_SLV2_ADDR 0x0B
#define ICM20948_I2C_SLV2_REG 0x0C
#define ICM20948_I2C_SLV2_CTRL 0x0D
#define ICM20948_I2C_SLV2_DO 0x0E
#define ICM20948_I2C_SLV3_ADDR 0x0F
#define ICM20948_I2C_SLV3_REG 0x10
#define ICM20948_I2C_SLV3_CTRL 0x11
#define ICM20948_I2C_SLV3_DO 0x12
#define ICM20948_I2C_SLV4_ADDR 0x013
#define ICM20948_I2C_SLV4_REG 0x14
#define ICM20948_I2C_SLV4_CTRL 0x15
#define ICM20948_I2C_SLV4_DO 0x16
#define ICM20948_I2C_SLV4_DI 0x17

#define AK09916_WIA2 0x01
#define AK09916_ST1 0x10
#define AK09916_HXL 0x11
#define AK09916_HXH 0x12
#define AK09916_HYL 0x13
#define AK09916_HYH 0x14
#define AK09916_HZL 0x15
#define AK09916_HZH 0x16
#define AK09916_ST2 0x18
#define AK09916_CNTL2 0x31
#define AK09916_CNTL3 0x32


/************** GYRO_CONFIG_1 **************/

enum GYRO_FS_SEL {
    GYRO_RANGE_250,
    GYRO_RANGE_500,
    GYRO_RANGE_1000,
    GYRO_RANGE_2000
};

enum GYRO_DLPFCFG {
    GYRO_DLPF_0, 
    GYRO_DLPF_1, 
    GYRO_DLPF_2, 
    GYRO_DLPF_3, 
    GYRO_DLPF_4, 
    GYRO_DLPF_5, 
    GYRO_DLPF_6, 
    GYRO_DLPF_7
};

enum GYRO_FCHOICE {
    GYRO_BYPASS_DLPF,
    GYRO_ENABLE_DLPF
};

enum ICM20948_ACCEL_RANGE {
    ICM20948_ACCEL_RANGE_2G,
    ICM20948_ACCEL_RANGE_4G,
    ICM20948_ACCEL_RANGE_8G,
    ICM20948_ACCEL_RANGE_16G
};

enum ICM20948_LOW_POW_GYRO_AVG {
    ICM20948_LOW_POW_GYRO_AVG_1,
    ICM20948_LOW_POW_GYRO_AVG_2,
    ICM20948_LOW_POW_GYRO_AVG_4,
    ICM20948_LOW_POW_GYRO_AVG_8,
    ICM20948_LOW_POW_GYRO_AVG_16,
    ICM20948_LOW_POW_GYRO_AVG_32,
    ICM20948_LOW_POW_GYRO_AVG_64,
    ICM20948_LOW_POW_GYRO_AVG_128
};

/************** USER_CTRL **************/

enum DMP_EN {
    DMP_DISABLED,
    DMP_ENABLED
};

enum FIFO_EN {
    FIFO_DISABLED,
    FIFO_ENABLED
};

enum I2C_MST_EN {
    I2C_DISABLED,
    I2C_ENABLED
};

enum I2C_IF_DIS {
    I2C_MODE,
    SPI_MODE
};

enum DMP_RST {
    DMP_RST_DO_NOTHING,
    RESET_DMP
};

enum SRAM_RST {
    SRAM_RST_DO_NOTHING,
    RESET_SRAM
};

enum I2C_MST_RST {
    I2C_RST_DO_NOTHING,
    RESET_I2C_MASTER
};

/************** LP_CONFIG **************/

enum I2C_MST_CYCLE {
    I2C_DUTY_CYCLE_MODE_DISABLED,
    I2C_DUTY_CYCLE_MODE_ENABLED
};

enum ACCEL_CYCLE {
    ACCEL_DUTY_CYCLE_MODE_DISABLED,
    ACCEL_DUTY_CYCLE_MODE_ENABLED
};

enum GYRO_CYCLE {
    GYRO_DUTY_CYCLE_MODE_DISABLED,
    GYRO_DUTY_CYCLE_MODE_ENABLED
};

/************** PWR_MGMT_1 **************/

enum DEVICE_RESET {
    DEVICE_RESET_DO_NOTHING,
    DEVICE_RESET_RESET_REGISTERS
};

enum SLEEP {
    SLEEP_MODE_DISABLED,
    SLEEP_MODE_ENABLED
};

enum LP_EN {
    LOW_POWER_MODE_DISABLED,
    LOW_POWER_MODE_ENABLED
};

enum TEMP_DIS {
    TEMP_SENSOR_DISABLED,
    TEMP_SENSOR_ENABLED
};

enum CLKSEL {
    INTERNAL_20MHZ_CLK = 0,
    AUTO_CLK = 1,
    STOP_CLK = 7
};

/************** PWR_MGMT_2 **************/

enum GYRO_STATUS {
    GYRO_DISABLE = 0x7,
    GYRO_ENABLE = 0
};

enum ACCEL_STATUS {
    ACCEL_DISABLE = 0x7,
    ACCEL_ENABLE = 0
};

/************** INT_PIN_CFG **************/


enum INT1_ACTL {
    INT1_LOGIC_HIGH,
    INT1_LOGIC_LOW
};

enum INT1_OPEN {
    INT1_PUSH_PULL,
    INT1_OPEN_DRAIN
};

enum INT1_LATCH__EN {
    INT1_INTERRUPT_PULSE_50US,
    INT1_HELD_UNTIL_CLEARED
};

enum INT1_ANYRD_2CLEAR {
    INT_STATUS_READ_CLEAR,
    ANY_READ_CLEAR
};

enum ACTL_FSYNC {
    FSYNC_ACTIVE_HIGH,
    FSYNC_ACTIVE_LOW
};

enum FSYNC_INT_MODE_EN {
    FSYNC_INT_DISABLE,
    FSYNC_INT_ENABLE
};

enum BYPASS_EN {
    I2C_MASTER_BYPASS_MODE_DISABLE,
    I2C_MASTER_BYPASS_MODE_ENABLE
};

/************** INT_ENABLE **************/


enum REG_WOF_EN {
    DISABLE_WAKE_FSYNC,
    ENABLE_WAKE_FSYNC
};

enum WOM_INT_EN {
    DISABLE_WAKE_MOTION,
    ENABLE_WAKE_MOTION
};

enum PLL_RDY_EN {
    DISABLE_PLL_RDY_INT,
    ENABLE_PLL_RDY_INT
};

enum DMP_INT1_EN {
    DISABLE_DMP_INT,
    ENABLE_DMP_INT
};

enum I2C_MST_INT_EN {
    DISABLE_I2C_MASTER_INT,
    ENABLE_I2C_MASTER_INT
};

/************** INT_ENABLE_1 **************/

enum RAW_DATA_0_RDY_EN {
    DISABLE_RAW_DATA_INT,
    ENABLE_RAW_DATA_INT
};

/************** INT_ENABLE_2 **************/

enum FIFO_OVERFLOW_EN {
    DISABLE_INT_FIFO_OVERFLOW,
    ENABLE_INT_FIFO_OVERFLOW = 0x1f
};

/************** INT_ENABLE_3 **************/

enum FIFO_WM_EN {
    DISABLE_INT_FIFO_WATERMARK,
    ENABLE_INT_FIFO_WATERMARK = 0x1f
};

/************** FIFO_EN_1 **************/

enum SLV_3_FIFO_EN {
    DISABLE_WRITE_TO_SLV_3,
    ENABLE_WRITE_TO_SLV_3
};

enum SLV_2_FIFO_EN {
    DISABLE_WRITE_TO_SLV_2,
    ENABLE_WRITE_TO_SLV_2
};

enum SLV_1_FIFO_EN {
    DISABLE_WRITE_TO_SLV_1,
    ENABLE_WRITE_TO_SLV_1
};

enum SLV_0_FIFO_EN {
    DISABLE_WRITE_TO_SLV_0,
    ENABLE_WIRTE_TO_SLV_0
};

/************** FIFO_EN_2 **************/


enum ACCEL_FIFO_EN {
    DISABLE_ACCEL_FIFO,
    ENABLE_ACCEL_FIFO
};

enum GYRO_Z_FIFO_EN {
    DISABLE_GYRO_Z_FIFO,
    ENABLE_GYRO_Z_FIFO
};

enum GYRO_Y_FIFO_EN {
    DISABLE_GYRO_Y_FIFO,
    ENABLE_GYRO_Y_FIFO
};

enum GYRO_X_FIFO_EN {
    DISABLE_GYRO_X_FIFO,
    ENABLE_GYRO_X_FIFO
};

enum TEMP_FIFO_EN {
    DISABLE_TEMP_FIFO,
    ENABLE_TEMP_FIFO
};

/************** FIFO_MODE **************/

enum FIFO_MODE {
    FIFO_STREAM,
    FIFO_SNAPSHOT
};

enum XGYRO_CTEN {
    X_GYRO_SELF_TEST_DISABLED,
    X_GYRO_SELF_TEST_ENABLED
};

enum YGYRO_CTEN {
    Y_GYRO_SELF_TEST_DISABLED,
    Y_GYRO_SELF_TEST_ENABLED
};

enum ZGYRO_CTEN {
    Z_GYRO_SELF_TEST_DISABLED,
    Z_GYRO_SELF_TEST_ENABLED
};

enum GYRO_AVGCFG {
    AVERAGING_1X,
    AVERAGING_2X,
    AVERAGING_4X,
    AVERAGING_8X,
    AVERAGING_16X,
    AVERAGING_32X,
    AVERAGING_64X,
    AVERAGING_128X
};

enum ODR_ALIGN_EN {
    DISABLE_ODR_START_TIME_ALIGNMENT,
    ENABLE_ODR_START_TIME_ALIGNMENT
};

enum ACCEL_INTEL_EN {
    WOM_LOGIC_DISABLED,
    WOM_LOGIC_ENABLED
};

enum ACCEL_INTEL_MODE_INT {
    INITIAL_SAMPLE,
    PREVIOUS_SAMPLE
};

/************** ACCEL_CONFIG **************/

enum ACCEL_DLPFCFG {
    ACCEL_DLPFCFG_0,
    ACCEL_DLPFCFG_1,
    ACCEL_DLPFCFG_2,
    ACCEL_DLPFCFG_3,
    ACCEL_DLPFCFG_4,
    ACCEL_DLPFCFG_5,
    ACCEL_DLPFCFG_6,
    ACCEL_DLPFCFG_8
};

enum ACCEL_FS_SEL {
    ACCEL_FULL_SCALE_2G,
    ACCEL_FULL_SCALE_4G,
    ACCEL_FULL_SCALE_8G,
    ACCEL_FULL_SCALE_16G
};

enum ACCEL_FCHOICE {
    ACCEL_DISABLE_DLPF,
    ACCEL_ENABLE_DLPF
};

enum AX_ST_EN_REG {
    X_ACCEL_SELF_TEST_DISABLED,
    X_ACCEL_SELF_TEST_ENABLED
};

enum AY_ST_EN_REG {
    Y_ACCEL_SELF_TEST_DISABLED,
    Y_ACCEL_SELF_TEST_ENABLED
};

enum AZ_ST_EN_REG {
    Z_ACCEL_SELF_TEST_DISABLED,
    Z_ACCEL_SELF_TEST_ENABLED
};

enum DEC3_CFG {
    AVERAGE_1_OR_4,
    AVERAGE_8,
    AVERAGE_16,
    AVERAGE_32
};

enum DELAY_TIME_EN {
    DISABLE_DELAY_TIME_MEASUREMENT,
    ENABLE_DELAY_TIME_MEASURMENT
};

enum WOF_DEGLITCH_EN {
    DISABLE_DIGITAL_DEGLITCHING,
    ENABLE_DIGITAL_DEGLITCHING
};

enum WOF_EDGE_INT {
    LEVEL_INTERRUPT,
    EDGE_INTERRUPT
};

enum EXT_SYNC_SET {
    EXT_SYNC_SET_DISABLED,
    TEMP_OUT_L,
    GYRO_XOUT_L,
    GYRO_YOUT_L,
    GYRO_ZOUT_L,
    ACCEL_XOUT_L,
    ACCEL_YOUT_L,
    ACCEL_ZOUT_L
};

enum TEMP_DLPFCFG {
    LPF_CONFIG_0,
    LPF_CONFIG_1,
    LPF_CONFIG_2,
    LPF_CONFIG_3,
    LPF_CONFIG_4,
    LPF_CONFIG_5,
    LPF_CONFIG_6,
    LPF_CONFIG_7
};

enum REG_LP_DMP_EN {
    DISABLE_DMP_LOW_POWER_ACCEL_MODE,
    ENABLE_DMP_LOW_POWER_ACCEL_MODE
};

enum AK09916_MODE {
    POWER_DOWN_MODE,
    SINGLE_MEASUREMENT_MODE,
    CONTINUOUS_MEASUREMENT_MODE1,
    CONTINUOUS_MEASUREMENT_MODE2,
    CONTINUOUS_MEASUREMENT_MODE3,
    CONTINUOUS_MEASUREMENT_MODE4,
    SELF_TEST_MODE
};

enum AK09916_SRST {
    NORMAL,
    RESET
};
// Structs

struct I2C_MST_STATUS {
    uint8_t PASS_THROUGH : 1;
    uint8_t I2C_SLV4_DONE : 1;
    uint8_t I2C_LOST_ARB : 1;
    uint8_t I2C_SLV4_NACK : 1;
    uint8_t I2C_SLV3_NACK : 1;
    uint8_t I2C_SLV2_NACK : 1;
    uint8_t I2C_SLV1_NACK : 1;
    uint8_t I2C_SLV0_NACK : 1;
};

struct INT_STATUS {
    uint8_t WOM_INT : 1;
    uint8_t PLL_RDY_INT : 1;
    uint8_t DMP_INT1 : 1;
    uint8_t I2C_MST_INT : 1;
};

struct INT_STATUS_1 {
    uint8_t RAW_DATA_0_RDY_INT : 1;
};

struct INT_STATUS_2 {
    uint8_t FIFO_OVERFLOW_INT : 1;
};

struct INT_STATUS_3 {
    uint8_t FIFO_WM_INT : 1;
};

struct DATA_RDY_STATUS {
    uint8_t WOF_STATUS : 1;
    uint8_t RAW_DATA_RDY : 1;
};

struct AK09916_STATUS1 {
    uint8_t DRDY : 1;
    uint8_t DOR : 1;
};

struct AK09916_STATUS2 {
    uint8_t HOFL : 1;
};

struct xyz16Int {
    uint16_t x;
    uint16_t y;
    uint16_t z;
};

struct accelGyroData {
    xyz16Int accel;
    xyz16Int gyro;
};

class ICM20948_IMU {
    public:
        ICM20948_IMU(mbed::SPI& spi_bus, mbed::DigitalOut& cs_pin, xyz16Int accel_offset, xyz16Int gyro_offset);

        void setUserCtrl(
            DMP_EN dmp_en, 
            FIFO_EN fifo_en,
            I2C_MST_EN i2c_mst_en, 
            I2C_IF_DIS i2c_if_dis, 
            DMP_RST dmp_rst, 
            SRAM_RST sram_rst, 
            I2C_MST_RST i2c_mst_rst
        );
        void setLpConfig(I2C_MST_CYCLE i2c_mst_cycle, ACCEL_CYCLE accel_cycle, GYRO_CYCLE gyro_cycle);
        void setPwrMgmt1(DEVICE_RESET device_reset, SLEEP sleep, LP_EN lp_en, TEMP_DIS temp_dis, CLKSEL clk_sel);
        void setPwrMgmt2(GYRO_STATUS gyro_status, ACCEL_STATUS accel_status);
        void setIntPinCfg(
            INT1_ACTL int1_actl, 
            INT1_OPEN int1_open, 
            INT1_LATCH__EN int1_latch_en, 
            INT1_ANYRD_2CLEAR int1_anyrd_2clear, 
            ACTL_FSYNC actl_fsync, 
            FSYNC_INT_MODE_EN fsync_int_mode_en,
            BYPASS_EN bypass_en
        );

        void setIntEnable(
            REG_WOF_EN reg_wof_en, 
            WOM_INT_EN wom_int_en, 
            PLL_RDY_EN pll_rdy_en, 
            DMP_INT1_EN dmp_int1_en, 
            I2C_MST_INT_EN i2c_mst_int_en
        );

        void setIntEnable1(RAW_DATA_0_RDY_EN raw_data_0_rdy_en);
        void setIntEnable2(FIFO_OVERFLOW_EN fifo_overflow_en);
        void setIntEnable3(FIFO_WM_EN fifo_wm_en);

        I2C_MST_STATUS i2cMstStatus();
        INT_STATUS intStatus();
        INT_STATUS_1 intStatus1();
        INT_STATUS_2 intStatus2();
        INT_STATUS_3 intStatus3();
        float delayTime();
        float tempOut();
        xyz16Int accelData();
        xyz16Int gyroData();
        accelGyroData readFifo();

        void setFifoEn1(
            SLV_3_FIFO_EN slv_3_fifo_en, 
            SLV_2_FIFO_EN slv_2_fifo_en, 
            SLV_1_FIFO_EN slv_1_fifo_en, 
            SLV_0_FIFO_EN slv_0_fifo_en
        );
        void setFifoEn2(
            ACCEL_FIFO_EN accel_fifo_en,
            GYRO_Z_FIFO_EN gyro_z_fifo_en,
            GYRO_Y_FIFO_EN gyro_y_fifo_en,
            GYRO_X_FIFO_EN gyro_x_fifo_en,
            TEMP_FIFO_EN temp_fifo_en
        );
        void fifoRst();
        void setFifoMode(FIFO_MODE fifo_mode);
        uint16_t getFifoCount();
        DATA_RDY_STATUS dataRdyStatus();
        xyz16Int accelOffsetOut();
        float timeBaseCorrectionPll();
        void setGyroSmplrtDiv(uint8_t smplrt);
        void setGyroConfig1(GYRO_DLPFCFG gyroDlpf, GYRO_FS_SEL gyroRange, GYRO_FCHOICE gyro_fchoice);
        void setGyroConfig2(XGYRO_CTEN x_gyro_cten, YGYRO_CTEN y_gyro_cten, ZGYRO_CTEN z_gyro_cten, GYRO_AVGCFG gyro_avgcf);
        void setOdrAlignEn(ODR_ALIGN_EN odr_align_en);
        void setAccelSmplrtDiv(uint16_t smplrt); 
        void setAccelIntelCtrl(ACCEL_INTEL_EN accel_intel_en, ACCEL_INTEL_MODE_INT accel_intel_mode_int);
        void setAccelWomThr(uint8_t wom_thr);
        void setAccelConfig(ACCEL_DLPFCFG accel_dlpfcfg, ACCEL_FS_SEL accel_fs_sel, ACCEL_FCHOICE accel_fchoice);
        void accelConfig2(AX_ST_EN_REG ax_st_en_reg, AY_ST_EN_REG ay_st_en_reg, AZ_ST_EN_REG az_st_en_reg, DEC3_CFG dec3_cfg);
        void fsyncConfig(DELAY_TIME_EN delay_time_en, WOF_DEGLITCH_EN wof_deglitch_en, WOF_EDGE_INT wof_edge_int, EXT_SYNC_SET ext_sync_set);
        void tempConfig(TEMP_DLPFCFG temp_dlpfcfg);
        void modCtrlUsr(REG_LP_DMP_EN reg_lp_dmp_en);
        AK09916_STATUS1 ak09916Status1();
        xyz16Int ak09916MeasurementData();
        AK09916_STATUS2 ak09916Status2();
        void ak09916Control2(AK09916_MODE mode);
        void ak09916Control3(AK09916_SRST srst);
        
    private:
        mbed::DigitalOut& cs_pin;
        uint8_t buf[12] = {0};
        static const float TEMP_SENSITIVITY;
        static const float TEMP_OFFSET;
        const xyz16Int accel_offset;
        const xyz16Int gyro_offset;

        mbed::SPI& spi_bus;
        uint8_t currentBank;
        void writeRegister8(uint8_t bank, uint8_t reg, uint8_t value);
        uint8_t readRegister8(uint8_t bank, uint8_t reg);
        void writeRegister16(uint8_t bank, uint8_t reg, uint16_t value);
        int16_t readRegister16(uint8_t bank, uint8_t reg);
        void readRegister48(uint8_t bank, uint8_t reg);
        void readRegister96(uint8_t bank, uint8_t reg);
        void switchBank(uint8_t bank);
        int16_t highLowByteTo16(uint8_t highByte, uint8_t lowByte);
        void setAccelOffsets();
        void setGyroOffsets();
};