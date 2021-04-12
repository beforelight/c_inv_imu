#include "drv_imu_invensense.h"

//#if defined(__cplusplus) || defined(c_plusplus)
//extern "C"{
//#endif

inv_imu_handle_t IMU_ConstructI2C(inv_i2c_t _i2c, uint8_t _addr) {
    inv_imu_handle_t rtv = INV_MALLOC(sizeof(inv_imu_t));
    memset(rtv, 0, sizeof(inv_imu_t));
    rtv->isSPI = false;
    rtv->i2c = _i2c;
    rtv->i2cTransfer.slaveAddress = _addr;
    rtv->i2cTransfer.subAddressSize = 1;
    if (_addr == IMU_SlaveAddressAutoDetect) {
        rtv->addrAutoDetect = true;
    } else {
        rtv->addrAutoDetect = false;
    }
    return rtv;
}
inv_imu_handle_t IMU_ConstructSPI(inv_spi_t _spi) {
    inv_imu_handle_t rtv = INV_MALLOC(sizeof(inv_imu_t));
    memset(rtv, 0, sizeof(inv_imu_t));
    rtv->isSPI = true;
    rtv->spi = _spi;
    return rtv;
}


int IMU_WriteReg(inv_imu_handle_t _this, uint8_t reg, uint8_t val) {
    int res = 0;
    if (!_this->isSPI) {
        _this->i2cTransfer.subAddress = reg;
        _this->i2cTransfer.data = &val;
        _this->i2cTransfer.dataSize = 1;
        _this->i2cTransfer.direction = inv_i2c_direction_Write;
        res = _this->i2c.masterTransferBlocking(&_this->i2cTransfer);
        if (res != 0) {
            INV_DEBUG("i2c write return code = %d", res);
        }
    } else {
        uint8_t txb[2];
        uint8_t rxb[2];
        txb[0] = (reg & 0x7fU);
        txb[1] = val;
        _this->spiTransfer.dataSize = 2;
        _this->spiTransfer.rxData = rxb;
        _this->spiTransfer.txData = txb;
        res = _this->spi.masterTransferBlocking(&_this->spiTransfer);
        if (res != 0) {
            INV_DEBUG("spi write return code = %d", res);
        }
    }
    return res;
}
int IMU_WriteRegVerified(inv_imu_handle_t _this, uint8_t reg, uint8_t val) {
    uint8_t regVal;
    int res = 0;
    res |= IMU_WriteReg(_this, reg, val);
    res |= IMU_ReadReg(_this, reg, &regVal);
    if (res == 0 && val != regVal) {
        res |= IMU_WriteReg(_this, reg, val);
        res |= IMU_ReadReg(_this, reg, &regVal);
        if (res == 0 && val != regVal) {
            INV_DEBUG("imu  rw error");
            res |= -1;
        }
    }
    return res;
}
int IMU_ReadReg(inv_imu_handle_t _this, uint8_t reg, uint8_t *val) {
    int res = 0;
    if (!_this->isSPI) {
        _this->i2cTransfer.subAddress = reg;
        _this->i2cTransfer.data = val;
        _this->i2cTransfer.dataSize = 1;
        _this->i2cTransfer.direction = inv_i2c_direction_Read;
        res = _this->i2c.masterTransferBlocking(&_this->i2cTransfer);
        if (res != 0) {
            INV_DEBUG("i2c read return code = %d", res);
        }
    } else {
        uint8_t txb[2];
        uint8_t rxb[2];
        txb[0] = (1U << 7U) | (reg & 0x7f);
        _this->spiTransfer.dataSize = 2;
        _this->spiTransfer.rxData = rxb;
        _this->spiTransfer.txData = txb;
        res = _this->spi.masterTransferBlocking(&_this->spiTransfer);
        if (res != 0) {
            INV_DEBUG("spi read return code = %d", res);
        } else {
            *val = rxb[1];
        }

    }
    return res;
}
int IMU_ModifyReg(inv_imu_handle_t _this, uint8_t reg, uint8_t val, uint8_t mask) {
    uint8_t regVal;
    int res = 0;
    res |= IMU_ReadReg(_this, reg, &regVal);
    res |= IMU_WriteRegVerified(_this, reg, (regVal & (~mask)) | (val & mask));
    res |= IMU_ReadReg(_this, reg, &regVal);
    if ((regVal & mask) != (val & mask)) {
        INV_DEBUG("imu rw error");
        res |= -1;
    }
    return res;
}


const float mpu_accel_fs_map_key[] = {
        MPU_FS_2G,
        MPU_FS_4G,
        MPU_FS_8G,
        MPU_FS_16G,
};
const int mpu_accel_fs_map_val[] = {
        0,
        1,
        2,
        3,
};
const struct _inv_weak_map_int_t mpu_accel_fs_map = {
        .key=mpu_accel_fs_map_key,
        .val =mpu_accel_fs_map_val,
        .n = sizeof(mpu_accel_fs_map_val) / sizeof(int)
};


const float mpu_gyro_fs_map_key[] = {
        MPU_FS_250dps,
        MPU_FS_500dps,
        MPU_FS_1000dps,
        MPU_FS_2000dps,
};
const int mpu_gyro_fs_map_val[] = {
        0,
        1,
        2,
        3,
};
const struct _inv_weak_map_int_t mpu_gyro_fs_map = {
        .key=mpu_gyro_fs_map_key,
        .val =mpu_gyro_fs_map_val,
        .n = sizeof(mpu_gyro_fs_map_val) / sizeof(int)
};


const float mpu_accel_unit_G_map_key[] = {
        MPU_FS_2G,
        MPU_FS_4G,
        MPU_FS_8G,
        MPU_FS_16G,
};
const float mpu_accel_unit_G_map_val[] = {
        2.0 / 32768.0,
        4.0 / 32768.0,
        8.0 / 32768.0,
        16.0 / 32768.0,
};
const struct _inv_weak_map_float_t mpu_accel_unit_G_map = {
        .key=mpu_accel_unit_G_map_key,
        .val =mpu_accel_unit_G_map_val,
        .n = sizeof(mpu_accel_unit_G_map_val) / sizeof(float)
};


const float mpu_gyro_unit_dps_map_key[] = {
        MPU_FS_250dps,
        MPU_FS_500dps,
        MPU_FS_1000dps,
        MPU_FS_2000dps,
};
const float mpu_gyro_unit_dps_map_val[] = {
        250.0 / 32768.0,
        500.0 / 32768.0,
        1000.0 / 32768.0,
        2000.0 / 32768.0,
};
const struct _inv_weak_map_float_t mpu_gyro_unit_dps_map = {
        .key=mpu_gyro_unit_dps_map_key,
        .val =mpu_gyro_unit_dps_map_val,
        .n = sizeof(mpu_gyro_unit_dps_map_val) / sizeof(float)
};


const float mpu_accel_unit_from_G_map_key[] = {
        MPU_UNIT_MetersPerSquareSecond,
        MPU_UNIT_G,
        MPU_UNIT_mG,
};
const float mpu_accel_unit_from_G_map_val[] = {
        9.8,
        1,
        1000
};
const struct _inv_weak_map_float_t mpu_accel_unit_from_G_map = {
        .key=mpu_accel_unit_from_G_map_key,
        .val =mpu_accel_unit_from_G_map_val,
        .n = sizeof(mpu_accel_unit_from_G_map_val) / sizeof(float)
};


const float mpu_gyro_unit_from_dps_map_key[] = {
        MPU_UNIT_DegPerSec,
        MPU_UNIT_RadPerSec,
        MPU_UNIT_RevolutionsPerMinute,
};
const float mpu_gyro_unit_from_dps_map_val[] = {
        1,
        3.14159265358979323846 / 180.0,
        60.0 / 360.0,
};
const struct _inv_weak_map_float_t mpu_gyro_unit_from_dps_map = {
        .key=mpu_gyro_unit_from_dps_map_key,
        .val =mpu_gyro_unit_from_dps_map_val,
        .n = sizeof(mpu_gyro_unit_from_dps_map_val) / sizeof(float)
};


const float MPU9250_GBW_MAP_key[] = {
        250,
        184,
        92,
        41,
        20,
        10,
        5,
};
const int MPU9250_GBW_MAP_val[] = {
        0,
        1,
        2,
        3,
        4,
        5,
        6,
};
const struct _inv_weak_map_int_t MPU9250_GBW_MAP = {
        .key=MPU9250_GBW_MAP_key,
        .val =MPU9250_GBW_MAP_val,
        .n = sizeof(MPU9250_GBW_MAP_val) / sizeof(int)
};


const float MPU9250_ABW_MAP_key[] = {
        218.1,
        99,
        44.8,
        21.2,
        10.2,
        5.05,
        420,
};
const int MPU9250_ABW_MAP_val[] = {
        1,
        2,
        3,
        4,
        5,
        6,
        7,
};
const struct _inv_weak_map_int_t MPU9250_ABW_MAP = {
        .key=MPU9250_ABW_MAP_key,
        .val =MPU9250_ABW_MAP_val,
        .n = sizeof(MPU9250_ABW_MAP_val) / sizeof(int)
};


const float ICM20948_GBW_MAP_key[] = {
        196.6,
        151.8,
        119.5,
        51.2,
        23.9,
        11.6,
        5.7,
        361.4,
};
const int ICM20948_GBW_MAP_val[] = {
        0,
        1,
        2,
        3,
        4,
        5,
        6,
        7,
};
const struct _inv_weak_map_int_t ICM20948_GBW_MAP = {
        .key=ICM20948_GBW_MAP_key,
        .val =ICM20948_GBW_MAP_val,
        .n = sizeof(ICM20948_GBW_MAP_val) / sizeof(int)
};
inv_imu_config_t IMU_ConfigDefault() {
    inv_imu_config_t result;
    result.gyroFullScale = MPU_FS_2000dps;
    result.gyroBandwidth = MPU_GBW_92;
    result.gyroUnit = MPU_UNIT_DegPerSec;
    result.accelFullScale = MPU_FS_8G;
    result.accelBandwidth = MPU_ABW_99;
    result.accelUnit = MPU_UNIT_MetersPerSquareSecond;


    return result;
}
void _IMU_Destruct(inv_imu_handle_t _this) { INV_FREE(_this); }
bool _IMU_IsOpen(inv_imu_handle_t _this) { return _this->isOpen; }


//#if defined(__cplusplus) || defined(c_plusplus)
//}
//#endif
