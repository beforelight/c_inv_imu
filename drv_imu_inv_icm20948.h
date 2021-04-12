#ifndef INV_IMU_INV_ICM20948_H
#define INV_IMU_INV_ICM20948_H
#include "drv_imu_invensense.h"
#if INV_ICM20948_ENABLE

//#if defined(__cplusplus) || defined(c_plusplus)
//extern "C"{
//#endif

typedef struct __inv_icm20948 {
    inv_imu parents;
    int bank;
    float gyroUnit;
    float accelUnit;
    uint8_t ak09916DeviceId;
    uint8_t *buf;
    uint8_t txbuf[24];
    uint8_t rxbuf[24];
} inv_icm20948, *inv_icm20948_handle;


inline void ICM20948_Destruct(inv_icm20948_handle _this) { _IMU_Destruct((void *) _this); }
inv_icm20948_handle ICM20948_ConstructI2C(inv_i2c _i2c, uint8_t _addr);
inv_icm20948_handle ICM20948_ConstructSPI(inv_spi _spi);


int ICM20948_Init(inv_icm20948_handle _this, inv_imu_config _cfg);
bool ICM20948_Detect(inv_icm20948_handle _this);
int ICM20948_SelfTest(inv_icm20948_handle _this);
inline const char *ICM20948_Report(inv_icm20948_handle _this) { return " icm20948"; }
bool ICM20948_DataReady(inv_icm20948_handle _this);
int ICM20948_EnableDataReadyInt(inv_icm20948_handle _this);
int ICM20948_SoftReset(inv_icm20948_handle _this);
int ICM20948_ReadSensorBlocking(inv_icm20948_handle _this);
int ICM20948_ReadSensorNonBlocking(inv_icm20948_handle _this);
int ICM20948_Convert(inv_icm20948_handle _this, float array[9]);
int ICM20948_ConvertRaw(inv_icm20948_handle _this, int16_t raw[9]);
int ICM20948_ConvertTemp(inv_icm20948_handle _this, float *temp);


int ICM20948_SubI2cRead(inv_icm20948_handle _this, uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len);
int ICM20948_SubI2cWrite(inv_icm20948_handle _this, uint8_t addr, uint8_t reg, const uint8_t *val, unsigned int len);
int ICM20948_WriteReg(inv_icm20948_handle _this, uint16_t reg, const uint8_t val);
int ICM20948_WriteRegVerified(inv_icm20948_handle _this, uint16_t reg, const uint8_t val);
int ICM20948_ReadReg(inv_icm20948_handle _this, uint16_t reg, uint8_t *val);
int ICM20948_ModifyReg(inv_icm20948_handle _this, uint16_t reg, const uint8_t val, const uint8_t mask);
int ICM20948_SwitchBank(inv_icm20948_handle _this, int _bank);


enum ICM20948_RegMap {
    ICM20948_WHO_AM_I = 0x0,//0      R
    ICM20948_USER_CTRL = 0x3,//3      R/W
    ICM20948_LP_CONFIG = 0x5,//5      R/W
    ICM20948_PWR_MGMT_1 = 0x6,//6      R/W
    ICM20948_PWR_MGMT_2 = 0x7,//7      R/W
    ICM20948_INT_PIN_CFG = 0x0F,//15     R/W
    ICM20948_INT_ENABLE = 0x10,//16     R/W
    ICM20948_INT_ENABLE_1 = 0x11,//17     R/W
    ICM20948_INT_ENABLE_2 = 0x12,//18     R/W
    ICM20948_INT_ENABLE_3 = 0x13,//19     R/W
    ICM20948_I2C_MST_STATUS = 0x17,//23     R/C
    ICM20948_INT_STATUS = 0x19,//25     R/C
    ICM20948_INT_STATUS_1 = 0x1A,//26     R/C
    ICM20948_INT_STATUS_2 = 0x1B,//27     R/C
    ICM20948_INT_STATUS_3 = 0x1C,//28     R/C
    ICM20948_DELAY_TIMEH = 0x28,//40     R
    ICM20948_DELAY_TIMEL = 0x29,//41     R
    ICM20948_ACCEL_XOUT_H = 0x2D,//45     R
    ICM20948_ACCEL_XOUT_L = 0x2E,//46     R
    ICM20948_ACCEL_YOUT_H = 0x2F,//47     R
    ICM20948_ACCEL_YOUT_L = 0x30,//48     R
    ICM20948_ACCEL_ZOUT_H = 0x31,//49     R
    ICM20948_ACCEL_ZOUT_L = 0x32,//50     R
    ICM20948_GYRO_XOUT_H = 0x33,//51     R
    ICM20948_GYRO_XOUT_L = 0x34,//52     R
    ICM20948_GYRO_YOUT_H = 0x35,//53     R
    ICM20948_GYRO_YOUT_L = 0x36,//54     R
    ICM20948_GYRO_ZOUT_H = 0x37,//55     R
    ICM20948_GYRO_ZOUT_L = 0x38,//56     R
    ICM20948_TEMP_OUT_H = 0x39,//57     R
    ICM20948_TEMP_OUT_L = 0x3A,//58     R
    ICM20948_EXT_SLV_SENS_DATA_00 = 0x3B,//59     R
    ICM20948_EXT_SLV_SENS_DATA_01 = 0x3C,//60     R
    ICM20948_EXT_SLV_SENS_DATA_02 = 0x3D,//61     R
    ICM20948_EXT_SLV_SENS_DATA_03 = 0x3E,//62     R
    ICM20948_EXT_SLV_SENS_DATA_04 = 0x3F,//63     R
    ICM20948_EXT_SLV_SENS_DATA_05 = 0x40,//64     R
    ICM20948_EXT_SLV_SENS_DATA_06 = 0x41,//65     R
    ICM20948_EXT_SLV_SENS_DATA_07 = 0x42,//66     R
    ICM20948_EXT_SLV_SENS_DATA_08 = 0x43,//67     R
    ICM20948_EXT_SLV_SENS_DATA_09 = 0x44,//68     R
    ICM20948_EXT_SLV_SENS_DATA_10 = 0x45,//69     R
    ICM20948_EXT_SLV_SENS_DATA_11 = 0x46,//70     R
    ICM20948_EXT_SLV_SENS_DATA_12 = 0x47,//71     R
    ICM20948_EXT_SLV_SENS_DATA_13 = 0x48,//72     R
    ICM20948_EXT_SLV_SENS_DATA_14 = 0x49,//73     R
    ICM20948_EXT_SLV_SENS_DATA_15 = 0x4A,//74     R
    ICM20948_EXT_SLV_SENS_DATA_16 = 0x4B,//75     R
    ICM20948_EXT_SLV_SENS_DATA_17 = 0x4C,//76     R
    ICM20948_EXT_SLV_SENS_DATA_18 = 0x4D,//77     R
    ICM20948_EXT_SLV_SENS_DATA_19 = 0x4E,//78     R
    ICM20948_EXT_SLV_SENS_DATA_20 = 0x4F,//79     R
    ICM20948_EXT_SLV_SENS_DATA_21 = 0x50,//80     R
    ICM20948_EXT_SLV_SENS_DATA_22 = 0x51,//81     R
    ICM20948_EXT_SLV_SENS_DATA_23 = 0x52,//82     R
    ICM20948_FIFO_EN_1 = 0x66,//102    R/W
    ICM20948_FIFO_EN_2 = 0x67,//103    R/W
    ICM20948_FIFO_RST = 0x68,//104    R/W
    ICM20948_FIFO_MODE = 0x69,//105    R/W
    ICM20948_FIFO_COUNTH = 0x70,//112    R
    ICM20948_FIFO_COUNTL = 0x71,//113    R
    ICM20948_FIFO_R_W = 0x72,//114    R/W
    ICM20948_DATA_RDY_STATUS = 0x74,//116    R/C
    ICM20948_FIFO_CFG = 0x76,//118    R/W
    ICM20948_REG_BANK_SEL = 0x7F,//127    R/W


    ICM20948_SELF_TEST_X_GYRO = (1 << 8) | 0x2,//2     R/W
    ICM20948_SELF_TEST_Y_GYRO = (1 << 8) | 0x3,//3     R/W
    ICM20948_SELF_TEST_Z_GYRO = (1 << 8) | 0x4,//4     R/W
    ICM20948_SELF_TEST_X_ACCEL = (1 << 8) | 0x0E,//14    R/W
    ICM20948_SELF_TEST_Y_ACCEL = (1 << 8) | 0x0F,//15    R/W
    ICM20948_SELF_TEST_Z_ACCEL = (1 << 8) | 0x10,//16    R/W
    ICM20948_XA_OFFS_H = (1 << 8) | 0x14,//20    R/W
    ICM20948_XA_OFFS_L = (1 << 8) | 0x15,//21    R/W
    ICM20948_YA_OFFS_H = (1 << 8) | 0x17,//23    R/W
    ICM20948_YA_OFFS_L = (1 << 8) | 0x18,//24    R/W
    ICM20948_ZA_OFFS_H = (1 << 8) | 0x1A,//26    R/W
    ICM20948_ZA_OFFS_L = (1 << 8) | 0x1B,//27    R/W
    ICM20948_TIMEBASE_CORRECTION_PLL = (1 << 8) | 0x28,//40    R/W
//        REG_BANK_SEL                 =(1<<8)|0x7F     ,//127   R/W



    ICM20948_GYRO_SMPLRT_DIV = (2 << 8) | 0x0,//0       R/W
    ICM20948_GYRO_CONFIG_1 = (2 << 8) | 0x1,//1       R/W
    ICM20948_GYRO_CONFIG_2 = (2 << 8) | 0x2,//2       R/W
    ICM20948_XG_OFFS_USRH = (2 << 8) | 0x3,//3       R/W
    ICM20948_XG_OFFS_USRL = (2 << 8) | 0x4,//4       R/W
    ICM20948_YG_OFFS_USRH = (2 << 8) | 0x5,//5       R/W
    ICM20948_YG_OFFS_USRL = (2 << 8) | 0x6,//6       R/W
    ICM20948_ZG_OFFS_USRH = (2 << 8) | 0x7,//7       R/W
    ICM20948_ZG_OFFS_USRL = (2 << 8) | 0x8,//8       R/W
    ICM20948_ODR_ALIGN_EN = (2 << 8) | 0x9,//9       R/W
    ICM20948_ACCEL_SMPLRT_DIV_1 = (2 << 8) | 0x10,//16      R/W
    ICM20948_ACCEL_SMPLRT_DIV_2 = (2 << 8) | 0x11,//17      R/W
    ICM20948_ACCEL_INTEL_CTRL = (2 << 8) | 0x12,//18      R/W
    ICM20948_ACCEL_WOM_THR = (2 << 8) | 0x13,//19      R/W
    ICM20948_ACCEL_CONFIG = (2 << 8) | 0x14,//20      R/W
    ICM20948_ACCEL_CONFIG_2 = (2 << 8) | 0x15,//21      R/W
    ICM20948_FSYNC_CONFIG = (2 << 8) | 0x52,//82      R/W
    ICM20948_TEMP_CONFIG = (2 << 8) | 0x53,//83      R/W
    ICM20948_MOD_CTRL_USR = (2 << 8) | 0x54,//84      R/W
//        REG_BANK_SEL                =(2<<8)|0x7F         ,//127     R/W



    ICM20948_I2C_MST_ODR_CONFIG = (3 << 8) | 0x0,//0      R/W
    ICM20948_I2C_MST_CTRL = (3 << 8) | 0x1,//1      R/W
    ICM20948_I2C_MST_DELAY_CTRL = (3 << 8) | 0x2,//2      R/W
    ICM20948_I2C_SLV0_ADDR = (3 << 8) | 0x3,//3      R/W
    ICM20948_I2C_SLV0_REG = (3 << 8) | 0x4,//4      R/W
    ICM20948_I2C_SLV0_CTRL = (3 << 8) | 0x5,//5      R/W
    ICM20948_I2C_SLV0_DO = (3 << 8) | 0x6,//6      R/W
    ICM20948_I2C_SLV1_ADDR = (3 << 8) | 0x7,//7      R/W
    ICM20948_I2C_SLV1_REG = (3 << 8) | 0x8,//8      R/W
    ICM20948_I2C_SLV1_CTRL = (3 << 8) | 0x9,//9      R/W
    ICM20948_I2C_SLV1_DO = (3 << 8) | 0x0A,//10     R/W
    ICM20948_I2C_SLV2_ADDR = (3 << 8) | 0x0B,//11     R/W
    ICM20948_I2C_SLV2_REG = (3 << 8) | 0x0C,//12     R/W
    ICM20948_I2C_SLV2_CTRL = (3 << 8) | 0x0D,//13     R/W
    ICM20948_I2C_SLV2_DO = (3 << 8) | 0x0E,//14     R/W
    ICM20948_I2C_SLV3_ADDR = (3 << 8) | 0x0F,//15     R/W
    ICM20948_I2C_SLV3_REG = (3 << 8) | 0x10,//16     R/W
    ICM20948_I2C_SLV3_CTRL = (3 << 8) | 0x11,//17     R/W
    ICM20948_I2C_SLV3_DO = (3 << 8) | 0x12,//18     R/W
    ICM20948_I2C_SLV4_ADDR = (3 << 8) | 0x13,//19     R/W
    ICM20948_I2C_SLV4_REG = (3 << 8) | 0x14,//20     R/W
    ICM20948_I2C_SLV4_CTRL = (3 << 8) | 0x15,//21     R/W
    ICM20948_I2C_SLV4_DO = (3 << 8) | 0x16,//22     R/W
    ICM20948_I2C_SLV4_DI = (3 << 8) | 0x17,//23     R
//        REG_BANK_SEL              =(3<<8)|0x7F      ,//127    R/W

};


enum AK09916_RegMap {
//Magnetometer register maps
    AK09916_WIA2 = 0x01,
    AK09916_ST1 = 0x10,
    AK09916_XOUT_L = 0x11,
    AK09916_XOUT_H = 0x12,
    AK09916_YOUT_L = 0x13,
    AK09916_YOUT_H = 0x14,
    AK09916_ZOUT_L = 0x15,
    AK09916_ZOUT_H = 0x16,
    AK09916_ST2 = 0x18,
    AK09916_CNTL2 = 0x31,
    AK09916_CNTL3 = 0x32,
    AK09916_TS1 = 0x33, //DO NOT ACCESS
    AK09916_TS2 = 0x34, //DO NOT ACCESS
};

//#if defined(__cplusplus) || defined(c_plusplus)
//}
//#endif

#endif //INV_XXX_ENABLE

#endif //INV_IMU_INV_ICM20948_H


