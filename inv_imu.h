//
// Created by 17616 on 2021/3/30.
//

#ifndef C_INV_IMU_INV_IMU_H
#define C_INV_IMU_INV_IMU_H
#include <stdint.h>
#include <float.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


#if !defined(INV_PRINTF)
#include<stdio.h>
#define INV_PRINTF printf
#endif //!defined(INV_PRINTF)

#ifdef INV_YES_TRACE
#define INV_TRACE_(fmt, ...) \
    INV_PRINTF("%s:%d:trace: " fmt "%s\r\n", __FILE__, __LINE__, __VA_ARGS__)
#define INV_TRACE(...) INV_TRACE_(__VA_ARGS__, "")
#else
#define INV_TRACE(...)
#endif //INV_YES_TRACE

#ifndef INV_NO_DEBUG
#define INV_DEBUG_(fmt, ...) \
    INV_PRINTF("%s:%d:debug: " fmt "%s\r\n", __FILE__, __LINE__, __VA_ARGS__)
#define INV_DEBUG(...) INV_DEBUG_(__VA_ARGS__, "")
#else
#define INV_DEBUG(...)
#endif //INV_NO_DEBUG


typedef enum __inv_i2c_direction {
    inv_i2c_direction_Write = 0U, /*!< Master transmit. */
    inv_i2c_direction_Read = 1U  /*!< Master receive. */
} inv_i2c_direction;

typedef struct __inv_i2c_transfer {
    uint16_t slaveAddress;
    uint8_t slaveAddressSize;
    uint8_t subAddressSize;
    uint32_t subAddress;
    void *data;
    uint32_t dataSize;
    inv_i2c_direction direction;
} inv_i2c_transfer;
typedef struct __inv_i2c {
    int (*masterTransferBlocking)(const inv_i2c_transfer *);
    int (*masterTransferNonBlocking)(const inv_i2c_transfer *);
} inv_i2c;


typedef struct __inv_spi_transfer {
    uint8_t *txData;          /*!< Send buffer. */
    uint8_t *rxData;          /*!< Receive buffer. */
    volatile uint32_t dataSize; /*!< Transfer bytes. */
} inv_spi_transfer;
typedef struct __inv_spi {
    int (*masterTransferBlocking)(const inv_spi_transfer *);
    int (*masterTransferNonBlocking)(const inv_spi_transfer *);
} inv_spi;


typedef struct __imu_config {
    enum mpu_accel_fs {
        MPU_FS_2G = 2,
        MPU_FS_4G = 4,
        MPU_FS_8G = 8,
        MPU_FS_16G = 16,
    } accelFullScale;

    enum mpu_accel_bw {
        MPU_ABW_420 = 420,
        MPU_ABW_218 = 218,
        MPU_ABW_99 = 99,
        MPU_ABW_45 = 45,
        MPU_ABW_21 = 21,
        MPU_ABW_10 = 10,
        MPU_ABW_5 = 5,
    } accelBandwidth;

    enum mpu_gyro_fs {
        MPU_FS_125dps = 125,
        MPU_FS_250dps = 250,
        MPU_FS_500dps = 500,
        MPU_FS_1000dps = 1000,
        MPU_FS_2000dps = 2000,
    } gyroFullScale;

    enum mpu_gyro_bw {
        MPU_GBW_361 = 361,
        MPU_GBW_250 = 250,
        MPU_GBW_176 = 176,
        MPU_GBW_92 = 92,
        MPU_GBW_41 = 41,
        MPU_GBW_20 = 20,
        MPU_GBW_10 = 10,
        MPU_GBW_5 = 5,
    } gyroBandwidth;

    enum mpu_gyro_unit {
        MPU_UNIT_DegPerSec,
        MPU_UNIT_RadPerSec,
        MPU_UNIT_RevolutionsPerMinute,
    } gyroUnit;

    enum mpu_accel_unit {
        MPU_UNIT_MetersPerSquareSecond,
        MPU_UNIT_G,
        MPU_UNIT_mG
    } accelUnit;
} inv_imu_config;

inv_imu_config IMU_ConfigDefault();

typedef struct __inv_imu_vector_table {
    int (*Init)(void *this, inv_imu_config _cfg);
    bool (*Detect)(void *this);
    int (*SelfTest)(void *this);
    const char *(*Report)(void *this);
    bool (*DataReady)(void *this);
    int (*EnableDataReadyInt)(void *this);
    int (*SoftReset)(void *this);
    int (*ReadSensorBlocking)(void *this);
    int (*ReadSensorNonBlocking)(void *this);
    int (*Convert)(void *this, float array[9]);
    int (*Convert2)(void *this, int16_t raw[9]);
    int (*Convert3)(void *this, float *temp);
    bool (*IsOpen)(void *this);
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


inline int IMU_Init(inv_imu_handle this, inv_imu_config _cfg) { return this->vtable->Init(this, _cfg); }
inline bool IMU_Detect(inv_imu_handle this) { return this->vtable->Detect(this); }
inline int IMU_SelfTest(inv_imu_handle this) { return this->vtable->SelfTest(this); }
inline const char *IMU_Report(inv_imu_handle this) { return this->vtable->Report(this); }
inline bool IMU_DataReady(inv_imu_handle this) { return this->vtable->DataReady(this); }
inline int IMU_EnableDataReadyInt(inv_imu_handle this) { return this->vtable->EnableDataReadyInt(this); }
inline int IMU_SoftReset(inv_imu_handle this) { return this->vtable->SoftReset(this); }
inline int IMU_ReadSensorBlocking(inv_imu_handle this) { return this->vtable->ReadSensorBlocking(this); }
inline int IMU_ReadSensorNonBlocking(inv_imu_handle this) { return this->vtable->ReadSensorNonBlocking(this); }
//顺序是 accel(xyz) gyro(xyz) mag(xyz)
inline int IMU_Convert(inv_imu_handle this, float array[9]) { return this->vtable->Convert(this, array); };
inline int IMU_Convert2(inv_imu_handle this, int16_t raw[9]) { return this->vtable->Convert2(this, raw); }
inline int IMU_Convert3(inv_imu_handle this, float *temp) { return this->vtable->Convert3(this, temp); }
inline bool IMU_IsOpen(inv_imu_handle this) { return this->vtable->IsOpen(this); }


#define  SlaveAddressAutoDetect 0
void IMU_Destruct(inv_imu_handle this) { free(this); }
inv_imu_handle IMU_Construct(inv_i2c _i2c, uint16_t _addr);
inv_imu_handle IMU_Construct2(inv_spi _spi);
int IMU_WriteReg(inv_imu_handle this, uint8_t reg, uint8_t val);
int IMU_WriteRegVerified(inv_imu_handle this, uint8_t reg, uint8_t val);
int IMU_ReadReg(inv_imu_handle this, uint8_t reg, uint8_t *val);
int IMU_ModifyReg(inv_imu_handle this, uint8_t reg, uint8_t val, uint8_t mask);
bool _IMU_IsOpen(inv_imu_handle this) { return this->isOpen; }


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

#endif //C_INV_IMU_INV_IMU_H
