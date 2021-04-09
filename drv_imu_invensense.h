#ifndef C_INV_IMU_INV_IMU_H
#define C_INV_IMU_INV_IMU_H
#include <inc_stdlib.h>
#include <hitsic_common.h>


#include <drv_imu_invensense_port.h>


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

#define kStatusGroup_INVIMU 300

enum
{
    kStatus_INVIMU_DetectFail = MAKE_STATUS(kStatusGroup_INVIMU, 0),
    kStatus_INVIMU_SelftestFail  = MAKE_STATUS(kStatusGroup_INVIMU, 1),
};

enum invimu_type_t 
{
    invimu_type_invalid = 0U,
    invimu_type_mpu6050_i2c,
    invimu_type_mpu9250_i2c,
    invimu_type_mpu9250_spi,
    invimu_type_icm20602_i2c,
    invimu_type_icm20602_spi,
    invimu_type_icm20948_i2c,
    invimu_type_icm20948_spi,
};

enum invimu_pptFlag_t
{
    invimu_pptFlag_dataInit = 1U << 0U,
    invimu_pptFlag_acceInit = 1U << 1U,
    invimu_pptFlag_gyroInit = 1U << 2U,
    invimu_pptFlag_magnInit = 1U << 3U,
};

typedef status_t(*invimu_spi_transfer_t)(uint8_t _regAddr, uint8_t *_data, uint32_t _size);

typedef status_t(*invimu_i2c_transfer_t)(uint8_t _devAddr, uint8_t _regAddr, uint8_t *_data, uint32_t _size);

typedef struct _invimu_spiBus
{
    invimu_spi_transfer_t rxBlocking;
    invimu_spi_transfer_t txBlocking;
}invimu_spiBus_t;

typedef struct _invimu_i2cBus
{
    invimu_i2c_transfer_t rxBlocking;
    invimu_i2c_transfer_t txBlocking;
}invimu_i2cBus_t;

typedef struct _invimu_config
{
    invimu_type_t type;
    union
    {
        invimu_spiBus_t spiBus;
        invimu_i2cBus_t i2cBus;
    }busHandle;
}invimu_config_t;

typedef struct _invimu_acceConfig
{

}invimu_acceConfig_t;

typedef struct _invimu_gyroConfig
{

}invimu_gyroConfig_t;

typedef struct _invimu_magnConfig
{

}invimu_acceConfig_t;

typedef struct _invimu_adapter invimu_adapter_t;

typedef struct _invimu_i2cIfce
{
    invimu_type_t type;

    uint32_t pptFlag;

    union 
    {
        invimu_default_handle_t         default_handle;
        invimu_mpu6050_i2c_handle_t*    mpu6050_i2c_handle;
        invimu_mpu9250_i2c_handle_t*    mpu9250_i2c_handle;
        invimu_icm20602_i2c_handle_t*   icm20602_i2c_handle;
        invimu_icm20948_i2c_handle_t*   icm20948_i2c_handle;
        invimu_mpu6000_spi_handle_t*    mpu6000_spi_handle;
        invimu_mpu9250_spi_handle_t*    mpu9250_spi_handle;
        invimu_icm20602_spi_handle_t*   icm20602_spi_handle;
        invimu_icm20948_spi_handle_t*   icm20948_spi_handle;
    } devHandle;

    const invimu_adapter_t* adapter;
}invimu_ifce_t;

struct _invimu_adapter
{
    void (*SoftReset)(invimu_ifce_t *_inst);
    status_t (*AcceInit)(invimu_ifce_t *_inst, invimu_acceConfig_t *_config);
    status_t (*GyroInit)(invimu_ifce_t *_inst, invimu_gyroConfig_t *_config);
    status_t (*MagnInit)(invimu_ifce_t *_inst, invimu_magnConfig_t *_config);

    status_t (*Selftest)(invimu_ifce_t *_inst);

    status_t (*AcceReadBlocking)(invimu_ifce_t *_inst);
    status_t (*GyroReadBlocking)(invimu_ifce_t *_inst);
    status_t (*MagnReadBlocking)(invimu_ifce_t *_inst);
    status_t (*TempReadBlocking)(invimu_ifce_t *_inst);

    void (*AcceConvertData)(invimu_ifce_t *_inst, float *data_x, float *data_y, float *data_z);
    void (*GyroConvertData)(invimu_ifce_t *_inst, float *data_x, float *data_y, float *data_z);
    void (*MagnConvertData)(invimu_ifce_t *_inst, float *data_x, float *data_y, float *data_z);
    void (*TempConvertData)(invimu_ifce_t *_inst, float *data);
};


static inline status_t INVIMU_DataInit(invimu_ifce_t *_inst, invimu_config_t *_config)
{
    assert(_inst);
    assert(_config);

    _inst->pptFlag = 0U;
    _inst->type = _config->type;

    switch(_inst->type)
    {
    case invimu_type_mpu6050_i2c:
        INVIMU_MPU6050_I2C_Construct(_inst->devHandle.mpu6050_i2c_handle, _config->busHandle.i2cBus);
        _inst->adapter = invimu_mpu6050_i2c_adapter;
        break;

    //TODO: add other type here.

    case invimu_type_invalid:
    default:
        _inst->type = invimu_type_invalid;
        _inst->devHandle->default_handle = NULL;
        _inst->adapter = NULL;
        break;

    }
    
}

static inline void INVIMU_DataDeinit(invimu_ifce_t *_inst)
{
    _inst->pptFlag = 0U;
    _inst->type = invimu_type_invalid;
    free(_inst->devHandle->default_handle);
    _inst->devHandle->default_handle = NULL;
    _inst->adapter = NULL;
    
}

static inline status_t INVIMU_SoftReset(invimu_ifce_t *_inst)
{
    _inst->adapter->SoftReset(_inst);
}

static inline status_t INVIMU_AcceInit(invimu_ifce_t *_inst, invimu_acceConfig_t *_config)
{
    _inst->adapter->AcceInit(_inst, _config);
}
static inline status_t INVIMU_GyroInit(invimu_ifce_t *_inst, invimu_gyroConfig_t *_config)
{
    _inst->adapter->GyroInit(_inst, _config);
}
static inline status_t INVIMU_MagnInit(invimu_ifce_t *_inst, invimu_magnConfig_t *_config)
{
    _inst->adapter->MagnInit(_inst, _config);
}

static inline status_t INVIMU_Selftest(invimu_ifce_t *_inst)
{
    _inst->adapter->Selftest(_inst);
}

static inline status_t INVIMU_AcceReadBlocking(invimu_ifce_t *_inst)
{
    _inst->adapter->AcceReadBlocking(_inst);
}
static inline status_t INVIMU_GyroReadBlocking(invimu_ifce_t *_inst)
{
    _inst->adapter->GyroReadBlocking(_inst);
}
static inline status_t INVIMU_MagnReadBlocking(invimu_ifce_t *_inst)
{
    _inst->adapter->MagnReadBlocking(_inst);
}
static inline status_t INVIMU_TempReadBlocking(invimu_ifce_t *_inst)
{
    _inst->adapter->TempReadBlocking(_inst);
}

static inline void INVIMU_AcceConvertData(invimu_ifce_t *_inst, float *data_x, float *data_y, float *data_z)
{
    _inst->adapter->AcceConvertData(_inst);
}
static inline void INVIMU_GyroConvertData(invimu_ifce_t *_inst, float *data_x, float *data_y, float *data_z)
{
    _inst->adapter->GyroConvertData(_inst);
}
static inline void INVIMU_MagnConvertData(invimu_ifce_t *_inst, float *data_x, float *data_y, float *data_z)
{
    _inst->adapter->MagnConvertData(_inst);
}
static inline void INVIMU_TempConvertData(invimu_ifce_t *_inst, float *data)
{
    _inst->adapter->TempConvertData(_inst);
}



/** MPU6050 */

typedef struct invimu_mpu6050_handle_t
{
    
};





#endif //C_INV_IMU_INV_IMU_H



