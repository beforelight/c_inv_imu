

#ifndef INV_IMU_INV_MPU6050_H
#define INV_IMU_INV_MPU6050_H
#include "drv_imu_invensense.h"
#if defined(INV_MPU6050_ENABLE)&&(INV_MPU6050_ENABLE>0U)

//#if defined(__cplusplus) || defined(c_plusplus)
//extern "C"{
//#endif

typedef struct __inv_mpu6050 {
    inv_imu parents;
    float gyroUnit;
    float accelUnit;
    uint8_t buf[14];
} inv_mpu6050, *inv_mpu6050_handle;


inline void MPU6050_Destruct(inv_mpu6050_handle _this) { _IMU_Destruct((void *) _this); }
inv_mpu6050_handle MPU6050_ConstructI2C(inv_i2c _i2c, uint8_t _addr);

int MPU6050_Init(inv_mpu6050_handle _this, inv_imu_config _cfg);
bool MPU6050_Detect(inv_mpu6050_handle _this);
int MPU6050_SelfTest(inv_mpu6050_handle _this);
inline const char *MPU6050_Report(inv_mpu6050_handle _this) { return "mpu6050"; }
bool MPU6050_DataReady(inv_mpu6050_handle _this);
int MPU6050_EnableDataReadyInt(inv_mpu6050_handle _this);
int MPU6050_SoftReset(inv_mpu6050_handle _this);
int MPU6050_ReadSensorBlocking(inv_mpu6050_handle _this);
int MPU6050_ReadSensorNonBlocking(inv_mpu6050_handle _this);
int MPU6050_Convert(inv_mpu6050_handle _this, float array[9]);
int MPU6050_ConvertRaw(inv_mpu6050_handle _this, int16_t raw[9]);
int MPU6050_ConvertTemp(inv_mpu6050_handle _this, float *temp);


enum MPU6050_RegMap {
    MPU6050_SELF_TEST_X = 0x0D,             //R/W
    MPU6050_SELF_TEST_Y = 0x0E,             //R/W
    MPU6050_SELF_TEST_Z = 0x0F,             //R/W
    MPU6050_SELF_TEST_A = 0x10,             //R/W
    MPU6050_SMPLRT_DIV = 0x19,             //R/W
    MPU6050_CONFIG = 0x1A,                 //R/W
    MPU6050_GYRO_CONFIG = 0x1B,             //R/W
    MPU6050_ACCEL_CONFIG = 0x1C,         //R/W
    MPU6050_FIFO_EN = 0x23,                 //R/W
    MPU6050_I2C_MST_CTRL = 0x24,         //R/W
    MPU6050_I2C_SLV0_ADDR = 0x25,         //R/W
    MPU6050_I2C_SLV0_REG = 0x26,         //R/W
    MPU6050_I2C_SLV0_CTRL = 0x27,         //R/W
    MPU6050_I2C_SLV1_ADDR = 0x28,         //R/W
    MPU6050_I2C_SLV1_REG = 0x29,         //R/W
    MPU6050_I2C_SLV1_CTRL = 0x2A,         //R/W
    MPU6050_I2C_SLV2_ADDR = 0x2B,         //R/W
    MPU6050_I2C_SLV2_REG = 0x2C,         //R/W
    MPU6050_I2C_SLV2_CTRL = 0x2D,         //R/W
    MPU6050_I2C_SLV3_ADDR = 0x2E,         //R/W
    MPU6050_I2C_SLV3_REG = 0x2F,         //R/W
    MPU6050_I2C_SLV3_CTRL = 0x30,         //R/W
    MPU6050_I2C_SLV4_ADDR = 0x31,         //R/W
    MPU6050_I2C_SLV4_REG = 0x32,         //R/W
    MPU6050_I2C_SLV4_DO = 0x33,             //R/W
    MPU6050_I2C_SLV4_CTRL = 0x34,         //R/W
    MPU6050_I2C_SLV4_DI = 0x35,             //R
    MPU6050_I2C_MST_STATUS = 0x36,         //R
    MPU6050_INT_PIN_CFG = 0x37,             //R/W
    MPU6050_INT_ENABLE = 0x38,             //R/W
    MPU6050_INT_STATUS = 0x3A,             //R
    MPU6050_ACCEL_XOUT_H = 0x3B,         //R
    MPU6050_ACCEL_XOUT_L = 0x3C,         //R
    MPU6050_ACCEL_YOUT_H = 0x3D,         //R
    MPU6050_ACCEL_YOUT_L = 0x3E,         //R
    MPU6050_ACCEL_ZOUT_H = 0x3F,         //R
    MPU6050_ACCEL_ZOUT_L = 0x40,         //R
    MPU6050_TEMP_OUT_H = 0x41,             //R
    MPU6050_TEMP_OUT_L = 0x42,             //R
    MPU6050_GYRO_XOUT_H = 0x43,             //R
    MPU6050_GYRO_XOUT_L = 0x44,             //R
    MPU6050_GYRO_YOUT_H = 0x45,             //R
    MPU6050_GYRO_YOUT_L = 0x46,             //R
    MPU6050_GYRO_ZOUT_H = 0x47,             //R
    MPU6050_GYRO_ZOUT_L = 0x48,             //R
    MPU6050_EXT_SENS_DATA_00 = 0x49,     //R
    MPU6050_EXT_SENS_DATA_01 = 0x4A,     //R
    MPU6050_EXT_SENS_DATA_02 = 0x4B,     //R
    MPU6050_EXT_SENS_DATA_03 = 0x4C,     //R
    MPU6050_EXT_SENS_DATA_04 = 0x4D,     //R
    MPU6050_EXT_SENS_DATA_05 = 0x4E,     //R
    MPU6050_EXT_SENS_DATA_06 = 0x4F,     //R
    MPU6050_EXT_SENS_DATA_07 = 0x50,     //R
    MPU6050_EXT_SENS_DATA_08 = 0x51,     //R
    MPU6050_EXT_SENS_DATA_09 = 0x52,     //R
    MPU6050_EXT_SENS_DATA_10 = 0x53,     //R
    MPU6050_EXT_SENS_DATA_11 = 0x54,     //R
    MPU6050_EXT_SENS_DATA_12 = 0x55,     //R
    MPU6050_EXT_SENS_DATA_13 = 0x56,     //R
    MPU6050_EXT_SENS_DATA_14 = 0x57,     //R
    MPU6050_EXT_SENS_DATA_15 = 0x58,     //R
    MPU6050_EXT_SENS_DATA_16 = 0x59,     //R
    MPU6050_EXT_SENS_DATA_17 = 0x5A,     //R
    MPU6050_EXT_SENS_DATA_18 = 0x5B,     //R
    MPU6050_EXT_SENS_DATA_19 = 0x5C,     //R
    MPU6050_EXT_SENS_DATA_20 = 0x5D,     //R
    MPU6050_EXT_SENS_DATA_21 = 0x5E,     //R
    MPU6050_EXT_SENS_DATA_22 = 0x5F,     //R
    MPU6050_EXT_SENS_DATA_23 = 0x60,     //R
    MPU6050_I2C_SLV0_DO = 0x63,             //R/W
    MPU6050_I2C_SLV1_DO = 0x64,             //R/W
    MPU6050_I2C_SLV2_DO = 0x65,             //R/W
    MPU6050_I2C_SLV3_DO = 0x66,             //R/W
    MPU6050_I2C_MST_DELAY_CTRL = 0x67,     //R/W
    MPU6050_SIGNAL_PATH_RESET = 0x68,     //R/W
    MPU6050_USER_CTRL = 0x6A,             //R/W
    MPU6050_PWR_MGMT_1 = 0x6B,             //R/W
    MPU6050_PWR_MGMT_2 = 0x6C,             //R/W
    MPU6050_FIFO_COUNTH = 0x72,             //R/W
    MPU6050_FIFO_COUNTL = 0x73,             //R/W
    MPU6050_FIFO_R_W = 0x74,             //R/W
    MPU6050_WHO_AM_I = 0x75,             //R
};

//#if defined(__cplusplus) || defined(c_plusplus)
//}
//#endif

#endif //INV_XXX_ENABLE
#endif //INV_IMU_INV_MPU6050_H



