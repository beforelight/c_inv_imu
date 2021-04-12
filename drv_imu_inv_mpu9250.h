

#ifndef INV_IMU_INV_MPU9250_H
#define INV_IMU_INV_MPU9250_H
#include "drv_imu_invensense.h"
#if INV_MPU9250_ENABLE

//#if defined(__cplusplus) || defined(c_plusplus)
//extern "C"{
//#endif

typedef struct __inv_mpu9250 {
    inv_imu parents;
    uint8_t *buf;
    uint8_t txbuf[23];
    uint8_t rxbuf[23];
    float gyroUnit;
    float accelUnit;
    uint8_t ak8963DeviceId;
    uint8_t ak8963Information;
    float ak8963Asa[3];
} inv_mpu9250, *inv_mpu9250_handle;


inline void MPU9250_Destruct(inv_mpu9250_handle _this) { _IMU_Destruct((void *) _this); }
inv_mpu9250_handle MPU9250_Construct(inv_i2c _i2c, uint8_t _addr);
inv_mpu9250_handle MPU9250_Construct2(inv_spi _spi);


int MPU9250_Init(inv_mpu9250_handle _this, inv_imu_config _cfg);
bool MPU9250_Detect(inv_mpu9250_handle _this);
int MPU9250_SelfTest(inv_mpu9250_handle _this);
inline const char *MPU9250_Report(inv_mpu9250_handle _this) { return "mpu9250"; }
bool MPU9250_DataReady(inv_mpu9250_handle _this);
int MPU9250_EnableDataReadyInt(inv_mpu9250_handle _this);
int MPU9250_SoftReset(inv_mpu9250_handle _this);
int MPU9250_ReadSensorBlocking(inv_mpu9250_handle _this);
int MPU9250_ReadSensorNonBlocking(inv_mpu9250_handle _this);
int MPU9250_Convert(inv_mpu9250_handle _this, float array[9]);
int MPU9250_Convert2(inv_mpu9250_handle _this, int16_t raw[9]);
int MPU9250_Convert3(inv_mpu9250_handle _this, float *temp);


int MPU9250_SubI2cRead(inv_mpu9250_handle _this, uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len);
int MPU9250_SubI2cWrite(inv_mpu9250_handle _this, uint8_t addr, uint8_t reg, const uint8_t *val, unsigned int len);


enum MPU9250_RegMap {
    MPU9250_SELF_TEST_X_GYRO = 0x0,//R/W
    MPU9250_SELF_TEST_Y_GYRO = 0x1,//R/W
    MPU9250_SELF_TEST_Z_GYRO = 0x2,//R/W
    MPU9250_SELF_TEST_X_ACCEL = 0x0D,//R/W
    MPU9250_SELF_TEST_Y_ACCEL = 0x0E,//R/W
    MPU9250_SELF_TEST_Z_ACCEL = 0x0F,//R/W
    MPU9250_XG_OFFSET_H = 0x13,//R/W
    MPU9250_XG_OFFSET_L = 0x14,//R/W
    MPU9250_YG_OFFSET_H = 0x15,//R/W
    MPU9250_YG_OFFSET_L = 0x16,//R/W
    MPU9250_ZG_OFFSET_H = 0x17,//R/W
    MPU9250_ZG_OFFSET_L = 0x18,//R/W
    MPU9250_SMPLRT_DIV = 0x19,//R/W
    MPU9250_CONFIG = 0x1A,//R/W
    MPU9250_GYRO_CONFIG = 0x1B,//R/W
    MPU9250_ACCEL_CONFIG = 0x1C,//R/W
    MPU9250_ACCEL_CONFIG2 = 0x1D,//R/W
    MPU9250_LP_ACCEL_ODR = 0x1E,//R/W
    MPU9250_WOM_THR = 0x1F,//R/W
    MPU9250_FIFO_EN = 0x23,//R/W
    MPU9250_I2C_MST_CTRL = 0x24,//R/W
    MPU9250_I2C_SLV0_ADDR = 0x25,//R/W
    MPU9250_I2C_SLV0_REG = 0x26,//R/W
    MPU9250_I2C_SLV0_CTRL = 0x27,//R/W
    MPU9250_I2C_SLV1_ADDR = 0x28,//R/W
    MPU9250_I2C_SLV1_REG = 0x29,//R/W
    MPU9250_I2C_SLV1_CTRL = 0x2A,//R/W
    MPU9250_I2C_SLV2_ADDR = 0x2B,//R/W
    MPU9250_I2C_SLV2_REG = 0x2C,//R/W
    MPU9250_I2C_SLV2_CTRL = 0x2D,//R/W
    MPU9250_I2C_SLV3_ADDR = 0x2E,//R/W
    MPU9250_I2C_SLV3_REG = 0x2F,//R/W
    MPU9250_I2C_SLV3_CTRL = 0x30,//R/W
    MPU9250_I2C_SLV4_ADDR = 0x31,//R/W
    MPU9250_I2C_SLV4_REG = 0x32,//R/W
    MPU9250_I2C_SLV4_DO = 0x33,//R/W
    MPU9250_I2C_SLV4_CTRL = 0x34,//R/W
    MPU9250_I2C_SLV4_DI = 0x35,//R
    MPU9250_I2C_MST_STATUS = 0x36,//R
    MPU9250_INT_PIN_CFG = 0x37,//R/W
    MPU9250_INT_ENABLE = 0x38,//R/W
    MPU9250_INT_STATUS = 0x3A,//R
    MPU9250_ACCEL_XOUT_H = 0x3B,//R
    MPU9250_ACCEL_XOUT_L = 0x3C,//R
    MPU9250_ACCEL_YOUT_H = 0x3D,//R
    MPU9250_ACCEL_YOUT_L = 0x3E,//R
    MPU9250_ACCEL_ZOUT_H = 0x3F,//R
    MPU9250_ACCEL_ZOUT_L = 0x40,//R
    MPU9250_TEMP_OUT_H = 0x41,//R
    MPU9250_TEMP_OUT_L = 0x42,//R
    MPU9250_GYRO_XOUT_H = 0x43,//R
    MPU9250_GYRO_XOUT_L = 0x44,//R
    MPU9250_GYRO_YOUT_H = 0x45,//R
    MPU9250_GYRO_YOUT_L = 0x46,//R
    MPU9250_GYRO_ZOUT_H = 0x47,//R
    MPU9250_GYRO_ZOUT_L = 0x48,//R
    MPU9250_EXT_SENS_DATA_00 = 0x49,//R
    MPU9250_EXT_SENS_DATA_01 = 0x4A,//R
    MPU9250_EXT_SENS_DATA_02 = 0x4B,//R
    MPU9250_EXT_SENS_DATA_03 = 0x4C,//R
    MPU9250_EXT_SENS_DATA_04 = 0x4D,//R
    MPU9250_EXT_SENS_DATA_05 = 0x4E,//R
    MPU9250_EXT_SENS_DATA_06 = 0x4F,//R
    MPU9250_EXT_SENS_DATA_07 = 0x50,//R
    MPU9250_EXT_SENS_DATA_08 = 0x51,//R
    MPU9250_EXT_SENS_DATA_09 = 0x52,//R
    MPU9250_EXT_SENS_DATA_10 = 0x53,//R
    MPU9250_EXT_SENS_DATA_11 = 0x54,//R
    MPU9250_EXT_SENS_DATA_12 = 0x55,//R
    MPU9250_EXT_SENS_DATA_13 = 0x56,//R
    MPU9250_EXT_SENS_DATA_14 = 0x57,//R
    MPU9250_EXT_SENS_DATA_15 = 0x58,//R
    MPU9250_EXT_SENS_DATA_16 = 0x59,//R
    MPU9250_EXT_SENS_DATA_17 = 0x5A,//R
    MPU9250_EXT_SENS_DATA_18 = 0x5B,//R
    MPU9250_EXT_SENS_DATA_19 = 0x5C,//R
    MPU9250_EXT_SENS_DATA_20 = 0x5D,//R
    MPU9250_EXT_SENS_DATA_21 = 0x5E,//R
    MPU9250_EXT_SENS_DATA_22 = 0x5F,//R
    MPU9250_EXT_SENS_DATA_23 = 0x60,//R
    MPU9250_I2C_SLV0_DO = 0x63,//R/W
    MPU9250_I2C_SLV1_DO = 0x64,//R/W
    MPU9250_I2C_SLV2_DO = 0x65,//R/W
    MPU9250_I2C_SLV3_DO = 0x66,//R/W
    MPU9250_I2C_MST_DELAY_CTRL = 0x67,//R/W
    MPU9250_SIGNAL_PATH_RESET = 0x68,//R/W
    MPU9250_MOT_DETECT_CTRL = 0x69,//R/W
    MPU9250_USER_CTRL = 0x6A,//R/W
    MPU9250_PWR_MGMT_1 = 0x6B,//R/W
    MPU9250_PWR_MGMT_2 = 0x6C,//R/W
    MPU9250_FIFO_COUNTH = 0x72,//R/W
    MPU9250_FIFO_COUNTL = 0x73,//R/W
    MPU9250_FIFO_R_W = 0x74,//R/W
    MPU9250_WHO_AM_I = 0x75,//R
    MPU9250_XA_OFFSET_H = 0x77,//R/W
    MPU9250_XA_OFFSET_L = 0x78,//R/W
    MPU9250_YA_OFFSET_H = 0x7A,//R/W
    MPU9250_YA_OFFSET_L = 0x7B,//R/W
    MPU9250_ZA_OFFSET_H = 0x7D,//R/W
    MPU9250_ZA_OFFSET_L = 0x7E,//R/W
};


enum AK8963_RegMap {
//Magnetometer register maps
    AK8963_WIA = 0x00,
    AK8963_INFO = 0x01,
    AK8963_ST1 = 0x02,
    AK8963_XOUT_L = 0x03,
    AK8963_XOUT_H = 0x04,
    AK8963_YOUT_L = 0x05,
    AK8963_YOUT_H = 0x06,
    AK8963_ZOUT_L = 0x07,
    AK8963_ZOUT_H = 0x08,
    AK8963_ST2 = 0x09,
    AK8963_CNTL = 0x0A,
    AK8963_CNTL2 = 0x0B,
    AK8963_RSV = 0x0B, //DO NOT ACCESS <MPU9250_AK8963_CNTL2>
    AK8963_ASTC = 0x0C,
    AK8963_TS1 = 0x0D, //DO NOT ACCESS
    AK8963_TS2 = 0x0E, //DO NOT ACCESS
    AK8963_I2CDIS = 0x0F,
    AK8963_ASAX = 0x10,
    AK8963_ASAY = 0x11,
    AK8963_ASAZ = 0x12,
};

//#if defined(__cplusplus) || defined(c_plusplus)
//}
//#endif

#endif //INV_XXX_ENABLE
#endif //INV_IMU_INV_MPU9250_H



