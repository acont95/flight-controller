#pragma once

#include <cstdint>
#include <spi_bus.hpp>
#include <gpio_output_interface.hpp>
#include <sleep_interface.hpp>
#include <math.h>

#define 	MIN(a, b)   ((b)>(a)?(a):(b))   

// #define ICM20948_USE_DMP
#define INV_MAX_SERIAL_WRITE 16
#define INV_MAX_SERIAL_READ 16

#if defined(ICM20948_USE_DMP)
    #define DMP_LOAD_ADDR 0x70
    const uint8_t dmp3_image[] = {
        #include "icm20948_img.dmp3a.h"
    };
#endif


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
#define DMP_MEM_START_ADDR 0x7C
#define DMP_MEM_R_W 0x7D
#define DMP_MEM_BANK_SEL 0x7E
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
#define ICM20948_YG_OFFS_USRL 0x06
#define ICM20948_ZG_OFFS_USRH 0x07
#define ICM20948_ZG_OFFS_USRL 0x08
#define ICM20948_ODR_ALIGN_EN 0x09
#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_ACCEL_INTEL_CTRL 0x12
#define ICM20948_ACCEL_WOM_THR 0x13
#define ICM20948_ACCEL_CONFIG 0x14
#define ICM20948_ACCEL_CONFIG_2 0x15
#define PRGM_STRT_ADDRH 0x50
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

// AK09916 REGISTER MAP
#define AK09916_DEVICE_ID 0x09
#define AK09916_SLAVE_ADDRESS 0x0C
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

/************** DMP_REGISTERS **************/
#define DMP_START_ADDRESS   ((uint16_t)0x1000)
// data output control
#define DATA_OUT_CTL1 (4 * 16)
#define DATA_OUT_CTL2 (4 * 16 + 2)
#define DATA_INTR_CTL (4 * 16 + 12)
#define FIFO_WATERMARK (31 * 16 + 14)
// motion event control
#define MOTION_EVENT_CTL (4 * 16 + 14)
// indicates to DMP which sensors are available
#define DMP_DATA_RDY_STATUS (8 * 16 + 10)
// batch mode
#define BM_BATCH_CNTR (27 * 16)
#define BM_BATCH_THLD (19 * 16 + 12)
#define BM_BATCH_MASK (21 * 16 + 14)
// sensor output data rate
#define ODR_ACCEL (11 * 16 + 14)
#define ODR_GYRO (11 * 16 + 10)
#define ODR_CPASS (11 * 16 + 6)
#define ODR_ALS (11 * 16 + 2)
#define ODR_QUAT6 (10 * 16 + 12)
#define ODR_QUAT9 (10 * 16 + 8)
#define ODR_PQUAT6 (10 * 16 + 4)
#define ODR_GEOMAG (10 * 16 + 0)
#define ODR_PRESSURE (11 * 16 + 12)
#define ODR_GYRO_CALIBR (11 * 16 + 8)
#define ODR_CPASS_CALIBR (11 * 16 + 4)
// sensor output data rate counter
#define ODR_CNTR_ACCEL (9 * 16 + 14)
#define ODR_CNTR_GYRO (9 * 16 + 10)
#define ODR_CNTR_CPASS (9 * 16 + 6)
#define ODR_CNTR_ALS (9 * 16 + 2)
#define ODR_CNTR_QUAT6 (8 * 16 + 12)
#define ODR_CNTR_QUAT9 (8 * 16 + 8)
#define ODR_CNTR_PQUAT6 (8 * 16 + 4)
#define ODR_CNTR_GEOMAG (8 * 16 + 0)
#define ODR_CNTR_PRESSURE (9 * 16 + 12)
#define ODR_CNTR_GYRO_CALIBR (9 * 16 + 8)
#define ODR_CNTR_CPASS_CALIBR (9 * 16 + 4)
// mounting matrix
#define CPASS_MTX_00 (23 * 16)
#define CPASS_MTX_01 (23 * 16 + 4)
#define CPASS_MTX_02 (23 * 16 + 8)
#define CPASS_MTX_10 (23 * 16 + 12)
#define CPASS_MTX_11 (24 * 16)
#define CPASS_MTX_12 (24 * 16 + 4)
#define CPASS_MTX_20 (24 * 16 + 8)
#define CPASS_MTX_21 (24 * 16 + 12)
#define CPASS_MTX_22 (25 * 16)
// bias calibration
#define GYRO_BIAS_X (139 * 16 + 4)
#define GYRO_BIAS_Y (139 * 16 + 8)
#define GYRO_BIAS_Z (139 * 16 + 12)
#define ACCEL_BIAS_X (110 * 16 + 4)
#define ACCEL_BIAS_Y (110 * 16 + 8)
#define ACCEL_BIAS_Z (110 * 16 + 12)
#define CPASS_BIAS_X (126 * 16 + 4)
#define CPASS_BIAS_Y (126 * 16 + 8)
#define CPASS_BIAS_Z (126 * 16 + 12)
// Accel FSR
#define ACC_SCALE (30 * 16 + 0)
#define ACC_SCALE2 (79 * 16 + 4)
// pedometer
#define PEDSTD_STEPCTR (54 * 16)
#define PEDSTD_TIMECTR (60 * 16 + 4)
// Activity Recognition
#define BAC_RATE (48 * 16 + 10)
// parameters for accel calibration
#define ACCEL_CAL_RATE (94 * 16 + 4)
#define ACCEL_ALPHA_VAR (91 * 16)
#define ACCEL_A_VAR (92 * 16)
// parameters for compass calibration
#define CPASS_TIME_BUFFER (112 * 16 + 14)
// gains
#define ACCEL_ONLY_GAIN (16 * 16 + 12)
#define GYRO_SF (19 * 16)

/************** GYRO_CONFIG_1 **************/

enum class GYRO_FS_SEL {
    GYRO_RANGE_250,
    GYRO_RANGE_500,
    GYRO_RANGE_1000,
    GYRO_RANGE_2000
};

enum class GYRO_DLPFCFG {
    GYRO_DLPF_0, 
    GYRO_DLPF_1, 
    GYRO_DLPF_2, 
    GYRO_DLPF_3, 
    GYRO_DLPF_4, 
    GYRO_DLPF_5, 
    GYRO_DLPF_6, 
    GYRO_DLPF_7
};

enum class GYRO_FCHOICE {
    GYRO_BYPASS_DLPF,
    GYRO_ENABLE_DLPF
};

enum class ICM20948_ACCEL_RANGE {
    ICM20948_ACCEL_RANGE_2G,
    ICM20948_ACCEL_RANGE_4G,
    ICM20948_ACCEL_RANGE_8G,
    ICM20948_ACCEL_RANGE_16G
};

enum class ICM20948_LOW_POW_GYRO_AVG {
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

enum class DMP_EN {
    DMP_DISABLED,
    DMP_ENABLED
};

enum class FIFO_EN {
    FIFO_DISABLED,
    FIFO_ENABLED
};

enum class I2C_MST_EN {
    I2C_DISABLED,
    I2C_ENABLED
};

enum class I2C_IF_DIS {
    I2C_MODE,
    SPI_MODE
};

enum class DMP_RST {
    DMP_RST_DO_NOTHING,
    RESET_DMP
};

enum class SRAM_RST {
    SRAM_RST_DO_NOTHING,
    RESET_SRAM
};

enum class I2C_MST_RST {
    I2C_RST_DO_NOTHING,
    RESET_I2C_MASTER
};

/************** LP_CONFIG **************/

enum class I2C_MST_CYCLE {
    I2C_DUTY_CYCLE_MODE_DISABLED,
    I2C_DUTY_CYCLE_MODE_ENABLED
};

enum class ACCEL_CYCLE {
    ACCEL_DUTY_CYCLE_MODE_DISABLED,
    ACCEL_DUTY_CYCLE_MODE_ENABLED
};

enum class GYRO_CYCLE {
    GYRO_DUTY_CYCLE_MODE_DISABLED,
    GYRO_DUTY_CYCLE_MODE_ENABLED
};

/************** PWR_MGMT_1 **************/

enum class DEVICE_RESET {
    DEVICE_RESET_DO_NOTHING,
    DEVICE_RESET_REGISTERS
};

enum class SLEEP {
    SLEEP_MODE_DISABLED,
    SLEEP_MODE_ENABLED
};

enum class LP_EN {
    LOW_POWER_MODE_DISABLED,
    LOW_POWER_MODE_ENABLED
};

enum class TEMP_DIS {
    TEMP_SENSOR_DISABLED,
    TEMP_SENSOR_ENABLED
};

enum class CLKSEL {
    INTERNAL_20MHZ_CLK = 0,
    AUTO_CLK = 1,
    STOP_CLK = 7
};

/************** PWR_MGMT_2 **************/

enum class GYRO_STATUS {
    GYRO_DISABLE = 0x7,
    GYRO_ENABLE = 0
};

enum class ACCEL_STATUS {
    ACCEL_DISABLE = 0x7,
    ACCEL_ENABLE = 0
};

/************** INT_PIN_CFG **************/


enum class INT1_ACTL {
    INT1_LOGIC_HIGH,
    INT1_LOGIC_LOW
};

enum class INT1_OPEN {
    INT1_PUSH_PULL,
    INT1_OPEN_DRAIN
};

enum class INT1_LATCH__EN {
    INT1_INTERRUPT_PULSE_50US,
    INT1_HELD_UNTIL_CLEARED
};

enum class INT1_ANYRD_2CLEAR {
    INT_STATUS_READ_CLEAR,
    ANY_READ_CLEAR
};

enum class ACTL_FSYNC {
    FSYNC_ACTIVE_HIGH,
    FSYNC_ACTIVE_LOW
};

enum class FSYNC_INT_MODE_EN {
    FSYNC_INT_DISABLE,
    FSYNC_INT_ENABLE
};

enum class BYPASS_EN {
    I2C_MASTER_BYPASS_MODE_DISABLE,
    I2C_MASTER_BYPASS_MODE_ENABLE
};

/************** INT_ENABLE **************/


enum class REG_WOF_EN {
    DISABLE_WAKE_FSYNC,
    ENABLE_WAKE_FSYNC
};

enum class WOM_INT_EN {
    DISABLE_WAKE_MOTION,
    ENABLE_WAKE_MOTION
};

enum class PLL_RDY_EN {
    DISABLE_PLL_RDY_INT,
    ENABLE_PLL_RDY_INT
};

enum class DMP_INT1_EN {
    DISABLE_DMP_INT,
    ENABLE_DMP_INT
};

enum class I2C_MST_INT_EN {
    DISABLE_I2C_MASTER_INT,
    ENABLE_I2C_MASTER_INT
};

/************** INT_ENABLE_1 **************/

enum class RAW_DATA_0_RDY_EN {
    DISABLE_RAW_DATA_INT,
    ENABLE_RAW_DATA_INT
};

/************** INT_ENABLE_2 **************/

enum class FIFO_OVERFLOW_EN {
    DISABLE_INT_FIFO_OVERFLOW,
    ENABLE_INT_FIFO_OVERFLOW
};

/************** INT_ENABLE_3 **************/

enum class FIFO_WM_EN {
    DISABLE_INT_FIFO_WATERMARK,
    ENABLE_INT_FIFO_WATERMARK
};

/************** FIFO_EN_1 **************/

enum class SLV_3_FIFO_EN {
    DISABLE_WRITE_TO_SLV_3,
    ENABLE_WRITE_TO_SLV_3
};

enum class SLV_2_FIFO_EN {
    DISABLE_WRITE_TO_SLV_2,
    ENABLE_WRITE_TO_SLV_2
};

enum class SLV_1_FIFO_EN {
    DISABLE_WRITE_TO_SLV_1,
    ENABLE_WRITE_TO_SLV_1
};

enum class SLV_0_FIFO_EN {
    DISABLE_WRITE_TO_SLV_0,
    ENABLE_WIRTE_TO_SLV_0
};

/************** FIFO_EN_2 **************/


enum class ACCEL_FIFO_EN {
    DISABLE_ACCEL_FIFO,
    ENABLE_ACCEL_FIFO
};

enum class GYRO_Z_FIFO_EN {
    DISABLE_GYRO_Z_FIFO,
    ENABLE_GYRO_Z_FIFO
};

enum class GYRO_Y_FIFO_EN {
    DISABLE_GYRO_Y_FIFO,
    ENABLE_GYRO_Y_FIFO
};

enum class GYRO_X_FIFO_EN {
    DISABLE_GYRO_X_FIFO,
    ENABLE_GYRO_X_FIFO
};

enum class TEMP_FIFO_EN {
    DISABLE_TEMP_FIFO,
    ENABLE_TEMP_FIFO
};

/************** FIFO_MODE **************/

enum class FIFO_MODE {
    FIFO_STREAM,
    FIFO_SNAPSHOT
};

enum class XGYRO_CTEN {
    X_GYRO_SELF_TEST_DISABLED,
    X_GYRO_SELF_TEST_ENABLED
};

enum class YGYRO_CTEN {
    Y_GYRO_SELF_TEST_DISABLED,
    Y_GYRO_SELF_TEST_ENABLED
};

enum class ZGYRO_CTEN {
    Z_GYRO_SELF_TEST_DISABLED,
    Z_GYRO_SELF_TEST_ENABLED
};

enum class GYRO_AVGCFG {
    AVERAGING_1X,
    AVERAGING_2X,
    AVERAGING_4X,
    AVERAGING_8X,
    AVERAGING_16X,
    AVERAGING_32X,
    AVERAGING_64X,
    AVERAGING_128X
};

enum class ODR_ALIGN_EN {
    DISABLE_ODR_START_TIME_ALIGNMENT,
    ENABLE_ODR_START_TIME_ALIGNMENT
};

enum class ACCEL_INTEL_EN {
    WOM_LOGIC_DISABLED,
    WOM_LOGIC_ENABLED
};

enum class ACCEL_INTEL_MODE_INT {
    INITIAL_SAMPLE,
    PREVIOUS_SAMPLE
};

/************** ACCEL_CONFIG **************/

enum class ACCEL_DLPFCFG {
    ACCEL_DLPFCFG_0,
    ACCEL_DLPFCFG_1,
    ACCEL_DLPFCFG_2,
    ACCEL_DLPFCFG_3,
    ACCEL_DLPFCFG_4,
    ACCEL_DLPFCFG_5,
    ACCEL_DLPFCFG_6,
    ACCEL_DLPFCFG_7
};

enum class ACCEL_FS_SEL {
    ACCEL_FULL_SCALE_2G,
    ACCEL_FULL_SCALE_4G,
    ACCEL_FULL_SCALE_8G,
    ACCEL_FULL_SCALE_16G
};

enum class ACCEL_FCHOICE {
    ACCEL_DISABLE_DLPF,
    ACCEL_ENABLE_DLPF
};

enum class AX_ST_EN_REG {
    X_ACCEL_SELF_TEST_DISABLED,
    X_ACCEL_SELF_TEST_ENABLED
};

enum class AY_ST_EN_REG {
    Y_ACCEL_SELF_TEST_DISABLED,
    Y_ACCEL_SELF_TEST_ENABLED
};

enum class AZ_ST_EN_REG {
    Z_ACCEL_SELF_TEST_DISABLED,
    Z_ACCEL_SELF_TEST_ENABLED
};

enum class DEC3_CFG {
    AVERAGE_1_OR_4,
    AVERAGE_8,
    AVERAGE_16,
    AVERAGE_32
};

enum class DELAY_TIME_EN {
    DISABLE_DELAY_TIME_MEASUREMENT,
    ENABLE_DELAY_TIME_MEASURMENT
};

enum class WOF_DEGLITCH_EN {
    DISABLE_DIGITAL_DEGLITCHING,
    ENABLE_DIGITAL_DEGLITCHING
};

enum class WOF_EDGE_INT {
    LEVEL_INTERRUPT,
    EDGE_INTERRUPT
};

enum class EXT_SYNC_SET {
    EXT_SYNC_SET_DISABLED,
    TEMP_OUT_L,
    GYRO_XOUT_L,
    GYRO_YOUT_L,
    GYRO_ZOUT_L,
    ACCEL_XOUT_L,
    ACCEL_YOUT_L,
    ACCEL_ZOUT_L
};

enum class TEMP_DLPFCFG {
    LPF_CONFIG_0,
    LPF_CONFIG_1,
    LPF_CONFIG_2,
    LPF_CONFIG_3,
    LPF_CONFIG_4,
    LPF_CONFIG_5,
    LPF_CONFIG_6,
    LPF_CONFIG_7
};

enum class REG_LP_DMP_EN {
    DISABLE_DMP_LOW_POWER_ACCEL_MODE,
    ENABLE_DMP_LOW_POWER_ACCEL_MODE
};

enum class AK09916_MODE {
    POWER_DOWN_MODE = 0x0,
    SINGLE_MEASUREMENT_MODE = 0x1,
    CONTINUOUS_MEASUREMENT_MODE1 = 0x2,
    CONTINUOUS_MEASUREMENT_MODE2 = 0x4,
    CONTINUOUS_MEASUREMENT_MODE3 = 0x6,
    CONTINUOUS_MEASUREMENT_MODE4 = 0x8,
    SELF_TEST_MODE = 0x10
};

enum class AK09916_SRST {
    NORMAL,
    RESET
};

/************** I2C_MST_CTRL **************/

enum class MULT_MST_EN {
    DISABLED,
    ENABLED
};

enum class I2C_MST_P_NSR {
    RESTART,
    STOP
};

enum class I2C_MST_CLK {
    CLK_0, //370.29 KHZ 50.00% Duty Cycle
    CLK_1, //- KHZ - Duty Cycle
    CLK_2, //370.29 KHZ 50.00% Duty Cycle
    CLK_3, //432.00 KHZ 50.00% Duty Cycle
    CLK_4, //370.29 KHZ 42.86% Duty Cycle
    CLK_5, //370.29 KHZ 50.00% Duty Cycle
    CLK_6, //345.60 KHZ 40.00% Duty Cycle 
    CLK_7, //345.60 KHZ 46.67% Duty Cycle 
    CLK_8, //304.94 KHZ 47.06% Duty Cycle 
    CLK_9, //432.00 KHZ 50.00% Duty Cycle 
    CLK_10, //432.00 KHZ 41.67% Duty Cycle 
    CLK_11, //432.00 KHZ 41.67% Duty Cycle 
    CLK_12, //471.27 KHZ 45.45% Duty Cycle 
    CLK_13, //432.00 KHZ 50.00% Duty Cycle
    CLK_14, //345.60 KHZ 46.67% Duty Cycle
    CLK_15 //345.60 KHZ 46.67% Duty Cycle
};


/************** I2C_MST_DELAY_CTRL ENUMS **************/

enum class DELAY_ES_SHADOW {
    DISABLED,
    ENABLED
};

enum class I2C_SLV4_DELAY_EN {
    DISABLED,
    ENABLED
};

enum class I2C_SLV3_DELAY_EN {
    DISABLED,
    ENABLED
};

enum class I2C_SLV2_DELAY_EN {
    DISABLED,
    ENABLED
};

enum class I2C_SLV1_DELAY_EN {
    DISABLED,
    ENABLED
};

enum class I2C_SLV0_DELAY_EN {
    DISABLED,
    ENABLED
};

/************** I2C_SLV0_REG ENUMS **************/

enum class I2C_SLV0_RNW {
    WRITE,
    READ
};

/************** I2C_SLV0_CTRL ENUMS **************/

enum class I2C_SLV0_EN {
    DISABLED,
    ENABLED
};

enum class I2C_SLV0_BYTE_SW {
    NO_SWAPPING,
    SWAP_BYTES
};

enum class I2C_SLV0_REG_DIS {
    DISABLED,
    READ_WRITE_ONLY
};

enum class I2C_SLV0_GRP {
    ODD_NUMBERING,
    EVEN_NUMBERING
};

// Structs

struct XYZ16Int {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct I2CMastStatus {
    uint8_t PASS_THROUGH;
    uint8_t I2C_SLV4_DONE;
    uint8_t I2C_LOST_ARB;
    uint8_t I2C_SLV4_NACK;
    uint8_t I2C_SLV3_NACK;
    uint8_t I2C_SLV2_NACK;
    uint8_t I2C_SLV1_NACK;
    uint8_t I2C_SLV0_NACK;
};

struct IntStatus {
    uint8_t WOM_INT;
    uint8_t PLL_RDY_INT;
    uint8_t DMP_INT1;
    uint8_t I2C_MST_INT;
};

struct IntStatus1 {
    uint8_t RAW_DATA_0_RDY_INT;
};

struct IntStatus2 {
    uint8_t FIFO_OVERFLOW_INT;
};

struct IntStatus3 {
    uint8_t FIFO_WM_INT;
};

struct DataRdyStatus {
    uint8_t WOF_STATUS;
    uint8_t RAW_DATA_RDY;
};

struct AK09916Status1 {
    uint8_t DRDY;
    uint8_t DOR;
};

struct AK09916Status2 {
    uint8_t HOFL;
};

struct AK09916Data {
    XYZ16Int mag;
    bool dataReady;
    bool overrun;
    bool overflow;
};

struct AccelGyroData {
    XYZ16Int accel;
    XYZ16Int gyro;
};

struct AccelGyroMagData {
    XYZ16Int accel;
    XYZ16Int gyro;
    AK09916Data mag_data;
};

struct GyroConfig1{
    GYRO_DLPFCFG gyro_dlpfcfg;
    GYRO_FS_SEL gyro_fs_sel;
    GYRO_FCHOICE gyro_fchoice;
};

struct AccelConfig {
    ACCEL_DLPFCFG accel_dlpfcfg;
    ACCEL_FS_SEL accel_fs_sel;
    ACCEL_FCHOICE accel_fchoice;
};

class ICM20948_IMU {
    public:
        ICM20948_IMU(SPIBusMaster& spi_bus, GPIOOutputInterface& cs_pin, SleepInterface& sleeper, XYZ16Int accel_offset, XYZ16Int gyro_offset);

        /************** USER_BANK_0_FUNCTIONS **************/
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
        I2CMastStatus getI2cMstStatus();
        IntStatus getIntStatus();
        IntStatus1 getIntStatus1();
        IntStatus2 getIntStatus2();
        IntStatus3 getIntStatus3();
        float getDelayTime();
        XYZ16Int getAccelData();
        XYZ16Int getGyroData();
        float getTempOut();
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
        void setFifoRst();
        void setFifoMode(FIFO_MODE fifo_mode);
        uint16_t getFifoCount();
        DataRdyStatus getDataRdyStatus();  

        /************** USER_BANK_1_FUNCTIONS **************/
        void setAccelOffsets();
        XYZ16Int getAccelOffsets();
        float getTimeBaseCorrectionPll();

        /************** USER_BANK_2_FUNCTIONS **************/
        void setGyroSmplrtDiv(uint8_t smplrt);
        void setGyroConfig1(GYRO_DLPFCFG gyroDlpf, GYRO_FS_SEL gyroRange, GYRO_FCHOICE gyro_fchoice);
        GyroConfig1 getGyroConfig1();
        void setGyroConfig2(XGYRO_CTEN x_gyro_cten, YGYRO_CTEN y_gyro_cten, ZGYRO_CTEN z_gyro_cten, GYRO_AVGCFG gyro_avgcf);
        void setGyroOffsets();
        void setOdrAlignEn(ODR_ALIGN_EN odr_align_en);
        void setAccelSmplrtDiv(uint16_t smplrt); 
        void setAccelIntelCtrl(ACCEL_INTEL_EN accel_intel_en, ACCEL_INTEL_MODE_INT accel_intel_mode_int);
        void setAccelWomThr(uint8_t wom_thr);
        void setAccelConfig(ACCEL_DLPFCFG accel_dlpfcfg, ACCEL_FS_SEL accel_fs_sel, ACCEL_FCHOICE accel_fchoice);
        AccelConfig getAccelConfig();
        void setAccelConfig2(AX_ST_EN_REG ax_st_en_reg, AY_ST_EN_REG ay_st_en_reg, AZ_ST_EN_REG az_st_en_reg, DEC3_CFG dec3_cfg);
        void setFsyncConfig(DELAY_TIME_EN delay_time_en, WOF_DEGLITCH_EN wof_deglitch_en, WOF_EDGE_INT wof_edge_int, EXT_SYNC_SET ext_sync_set);
        void setTempConfig(TEMP_DLPFCFG temp_dlpfcfg);
        void setModCtrlUsr(REG_LP_DMP_EN reg_lp_dmp_en);

        /************** USER_BANK_3_FUNCTIONS **************/
        void setI2cMstOdrConfig(uint8_t smplrt);
        void setI2cMstCtrl(MULT_MST_EN mult_mst_en, I2C_MST_P_NSR i2c_mst_p_nsr, I2C_MST_CLK i2c_mst_clk);
        void setI2cMstDelayCtrl(DELAY_ES_SHADOW delay_es_shadow, I2C_SLV4_DELAY_EN i2c_slv4_delay_en, I2C_SLV3_DELAY_EN i2c_slv3_delay_en,
            I2C_SLV2_DELAY_EN i2c_slv2_delay_en, I2C_SLV1_DELAY_EN i2c_slv1_delay_en, I2C_SLV0_DELAY_EN i2c_slv0_delay_en);
        void setI2cSlv0Addr(I2C_SLV0_RNW i2c_slv0_rnw, uint8_t i2c_id_0);
        void setI2cSlv0Reg(uint8_t reg);
        void setI2cSlv0Ctrl(I2C_SLV0_EN i2c_slv0_en, I2C_SLV0_BYTE_SW i2c_slv0_byte_sw, I2C_SLV0_REG_DIS i2c_slv0_reg_dis, I2C_SLV0_GRP i2c_slv0_grp, uint8_t i2c_slvo_leng);
        void setI2cSlv0Do(uint8_t data);

        /************** MAGNETOMETER_FUNCTIONS **************/
        AK09916Status1 getAk09916Status1();
        void setupAk09916MeasurementData();
        AK09916Status2 getAk09916Status2();
        void setAk09916Control2(AK09916_MODE mode);
        void setAk09916Control3(AK09916_SRST srst);
        uint8_t getAk09916WhoAmI();
        void ak09916Init();
        uint8_t readSingleAK09916Reg(uint8_t reg);
        void readAk09916Reg(uint8_t reg, uint8_t len);
        AK09916Data readAk09916ExtData();
        void writeSingleAK09916Reg(uint8_t reg, uint8_t val);

        /************** DMP_FUNCTIONS **************/
        void loadDmpFirmware(uint8_t dmp3_image[], uint16_t dmp3_image_size, uint8_t load_addr);
        void setDmpStartAddress();

        /************** HELPER_FUNCTIONS **************/
        float getGyroScaleFactor(GYRO_FS_SEL gyro_full_scale);
        float getAccelScaleFactor(ACCEL_FS_SEL accel_full_scale);
        AccelGyroData readFifo();
        AccelGyroMagData readFifoAllSensors();
        void resetI2cMaster();
        void enableI2cMaster();
        uint8_t buf[32] = {0};

    private:
        SPIBusMaster& spi_bus;
        GPIOOutputInterface& cs_pin;
        SleepInterface& sleeper;
        // uint8_t buf[32] = {0};
        static const float TEMP_SENSITIVITY;
        static const float TEMP_OFFSET;
        const XYZ16Int accel_offset;
        const XYZ16Int gyro_offset;

        uint8_t currentBank = 0;
        uint8_t currentDmpBank = 0;
        void writeRegister8(uint8_t bank, uint8_t reg, uint8_t value);
        uint8_t readRegister8(uint8_t bank, uint8_t reg);
        void writeRegister16(uint8_t bank, uint8_t reg, uint16_t value);
        int16_t readRegister16(uint8_t bank, uint8_t reg);
        void readRegister48(uint8_t bank, uint8_t reg);
        void writeRegister(uint8_t bank, uint8_t reg, uint8_t data[], uint8_t size);
        void readRegister(uint8_t bank, uint8_t reg, uint8_t data[], uint8_t size);
        void switchBank(uint8_t bank);
        void switchDmpBank(uint8_t bank);
        int16_t highLowByteTo16(uint8_t highByte, uint8_t lowByte);
};