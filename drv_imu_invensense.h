﻿#ifndef C_INV_IMU_INV_IMU_H
#define C_INV_IMU_INV_IMU_H
#ifdef _C_INV_IMU_DEV_
#include "drv_imu_invensense_port_template.h"
#else
#include "../drv_imu_invensense_port.h"
#endif
#include<stdint.h>
#include<stdbool.h>
#include<string.h>
#include<float.h>

//处理未定义宏定义
#ifndef INV_MPU6050_ENABLE
#define INV_MPU6050_ENABLE 0
#endif
#ifndef INV_MPU9250_ENABLE
#define INV_MPU9250_ENABLE  0
#endif
#ifndef INV_ICM20602_ENABLE
#define INV_ICM20602_ENABLE 0
#endif
#ifndef INV_ICM20600_ENABLE
#define INV_ICM20600_ENABLE 0
#endif
#ifndef INV_ICM20948_ENABLE
#define INV_ICM20948_ENABLE 0
#endif
#if !defined(INV_MALLOC)||!defined(INV_FREE)||!defined(INV_REALLOC)
#include<stdlib.h>
#define INV_MALLOC malloc
#define INV_FREE free
#define INV_REALLOC realloc
#endif

#ifndef INV_ASSERT
#define INV_ASSERT(x) x
#endif
#ifndef INV_ERROR
#define INV_ERROR(x)  x
#endif
#ifndef INV_INFO
#define INV_INFO(x)   x
#endif
#ifndef INV_DEBUG
#define INV_DEBUG(x)  x
#endif



enum mpu_accel_fs {
    MPU_FS_2G = 2,
    MPU_FS_4G = 4,
    MPU_FS_8G = 8,
    MPU_FS_16G = 16,
};
enum mpu_accel_bw {
    MPU_ABW_420 = 420,
    MPU_ABW_218 = 218,
    MPU_ABW_99 = 99,
    MPU_ABW_45 = 45,
    MPU_ABW_21 = 21,
    MPU_ABW_10 = 10,
    MPU_ABW_5 = 5,
};
enum mpu_gyro_fs {
    MPU_FS_125dps = 125,
    MPU_FS_250dps = 250,
    MPU_FS_500dps = 500,
    MPU_FS_1000dps = 1000,
    MPU_FS_2000dps = 2000,
};
enum mpu_gyro_bw {
    MPU_GBW_361 = 361,
    MPU_GBW_250 = 250,
    MPU_GBW_176 = 176,
    MPU_GBW_92 = 92,
    MPU_GBW_41 = 41,
    MPU_GBW_20 = 20,
    MPU_GBW_10 = 10,
    MPU_GBW_5 = 5,
};
enum mpu_gyro_unit {
    MPU_UNIT_DegPerSec,
    MPU_UNIT_RadPerSec,
    MPU_UNIT_RevolutionsPerMinute,
};
enum mpu_accel_unit {
    MPU_UNIT_MetersPerSquareSecond,
    MPU_UNIT_G,
    MPU_UNIT_mG
};
typedef struct __imu_config {
    enum mpu_accel_fs accelFullScale;
    enum mpu_accel_bw accelBandwidth;
    enum mpu_gyro_fs gyroFullScale;
    enum mpu_gyro_bw gyroBandwidth;
    enum mpu_gyro_unit gyroUnit;
    enum mpu_accel_unit accelUnit;
} inv_imu_config;
inv_imu_config IMU_ConfigDefault();

typedef struct __inv_imu_vector_table {
    int (*Init)(void *_this, inv_imu_config _cfg);
    bool (*Detect)(void *_this);
    int (*SelfTest)(void *_this);
    const char *(*Report)(void *_this);
    bool (*DataReady)(void *_this);
    int (*EnableDataReadyInt)(void *_this);
    int (*SoftReset)(void *_this);
    int (*ReadSensorBlocking)(void *_this);
    int (*ReadSensorNonBlocking)(void *_this);
    int (*Convert)(void *_this, float array[9]);
    int (*Convert2)(void *_this, int16_t raw[9]);
    int (*Convert3)(void *_this, float *temp);
    bool (*IsOpen)(void *_this);
    void (*Destruct)(void *_this);
} inv_imu_vector_table;

typedef struct __inv_imu {
    inv_imu_vector_table *vtable;
    inv_i2c i2c;
    inv_spi spi;
    inv_i2c_transfer i2cTransfer;
    inv_spi_transfer spiTransfer;
    bool addrAutoDetect;
    bool isOpen;
    inv_imu_config cfg;
} inv_imu, *inv_imu_handle;


inline int IMU_Init(inv_imu_handle _this, inv_imu_config _cfg) { return _this->vtable->Init(_this, _cfg); }
inline bool IMU_Detect(inv_imu_handle _this) { return _this->vtable->Detect(_this); }
inline int IMU_SelfTest(inv_imu_handle _this) { return _this->vtable->SelfTest(_this); }
inline const char *IMU_Report(inv_imu_handle _this) { return _this->vtable->Report(_this); }
inline bool IMU_DataReady(inv_imu_handle _this) { return _this->vtable->DataReady(_this); }
inline int IMU_EnableDataReadyInt(inv_imu_handle _this) { return _this->vtable->EnableDataReadyInt(_this); }
inline int IMU_SoftReset(inv_imu_handle _this) { return _this->vtable->SoftReset(_this); }
inline int IMU_ReadSensorBlocking(inv_imu_handle _this) { return _this->vtable->ReadSensorBlocking(_this); }
inline int IMU_ReadSensorNonBlocking(inv_imu_handle _this) { return _this->vtable->ReadSensorNonBlocking(_this); }
//顺序是 accel(xyz) gyro(xyz) mag(xyz)
inline int IMU_Convert(inv_imu_handle _this, float array[9]) { return _this->vtable->Convert(_this, array); };
inline int IMU_Convert2(inv_imu_handle _this, int16_t raw[9]) { return _this->vtable->Convert2(_this, raw); }
inline int IMU_Convert3(inv_imu_handle _this, float *temp) { return _this->vtable->Convert3(_this, temp); }
inline bool IMU_IsOpen(inv_imu_handle _this) { return _this->vtable->IsOpen(_this); }
inline void IMU_Destruct(inv_imu_handle _this) { return _this->vtable->Destruct(_this); }

const int IMU_SlaveAddressAutoDetect = 0;
inline void _IMU_Destruct(inv_imu_handle _this) { INV_FREE(_this); }
inv_imu_handle IMU_Construct(inv_i2c _i2c, uint16_t _addr);
inv_imu_handle IMU_Construct2(inv_spi _spi);
int IMU_WriteReg(inv_imu_handle _this, uint8_t reg, uint8_t val);
int IMU_WriteRegVerified(inv_imu_handle _this, uint8_t reg, uint8_t val);
int IMU_ReadReg(inv_imu_handle _this, uint8_t reg, uint8_t *val);
int IMU_ModifyReg(inv_imu_handle _this, uint8_t reg, uint8_t val, uint8_t mask);
inline bool _IMU_IsOpen(inv_imu_handle _this) { return _this->isOpen; }
inv_imu_handle IMU_AutoConstruct(inv_i2c _i2c, uint16_t _addr);
inv_imu_handle IMU_AutoConstruct2(inv_spi _spi);


struct _inv_weak_map_int {
    const float *key;
    const int *val;
    int n;
};

struct _inv_weak_map_float {
    const float *key;
    const float *val;
    int n;
};

#define _InvGetMapVal(map, _key, the_val) { \
int n = 0;\
float distance = FLT_MAX;\
float buf;\
for (int i = 0; i < map.n; ++i) {\
buf = map.key[i] - _key;\
buf *= buf;\
if (buf < distance) {\
distance = buf;\
n = i;\
}\
}  the_val = map.val[n];}


extern const struct _inv_weak_map_int mpu_accel_fs_map;
extern const struct _inv_weak_map_int mpu_gyro_fs_map;
extern const struct _inv_weak_map_float mpu_accel_unit_G_map;
extern const struct _inv_weak_map_float mpu_gyro_unit_dps_map;
extern const struct _inv_weak_map_float mpu_accel_unit_from_G_map;
extern const struct _inv_weak_map_float mpu_gyro_unit_from_dps_map;
extern const struct _inv_weak_map_int MPU9250_GBW_MAP;
extern const struct _inv_weak_map_int MPU9250_ABW_MAP;
extern const struct _inv_weak_map_int ICM20948_GBW_MAP;

#define MPU6050_GBW_MAP   MPU9250_GBW_MAP
#define ICM20602_GBW_MAP  MPU9250_GBW_MAP
#define ICM20602_ABW_MAP  MPU9250_ABW_MAP
#define ICM20948_ABW_MAP  MPU9250_ABW_MAP

//#if defined(__cplusplus) || defined(c_plusplus)
//}
//#endif

#endif //C_INV_IMU_INV_IMU_H


