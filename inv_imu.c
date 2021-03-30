//
// Created by 17616 on 2021/3/30.
//

#include "inv_imu.h"
int IMU_WriteReg(inv_imu_handle this, uint8_t reg, uint8_t val) {
    int res = 0;
    if (this->i2c != NULL) {
        this->i2cTransfer.subAddress = reg;
        this->i2cTransfer.data = &val;
        this->i2cTransfer.dataSize = 1;
        this->i2cTransfer.direction = inv_i2c_direction_Write;
        res = this->i2c->masterTransferBlocking(&this->i2cTransfer);
        if (res != 0) {
            INV_DEBUG("i2c write return code = %d", res);
        }
    } else {
        uint8_t txb[2];
        uint8_t rxb[2];
        txb[0] = (reg & 0x7fU);
        txb[1] = val;
        this->spiTransfer.dataSize = 2;
        this->spiTransfer.rxData = rxb;
        this->spiTransfer.txData = txb;
        res = this->spi->masterTransferBlocking(&this->spiTransfer);
        if (res != 0) {
            INV_DEBUG("spi write return code = %d", res);
        }
    }
    return res;
}
int IMU_WriteRegVerified(inv_imu_handle this, uint8_t reg, uint8_t val) {
    uint8_t regVal;
    int res = 0;
    res |= IMU_WriteReg(this, reg, val);
    res |= IMU_ReadReg(this, reg, &regVal);
    if (res == 0 && val != regVal) {
        res |= IMU_WriteReg(this, reg, val);
        res |= IMU_ReadReg(this, reg, &regVal);
        if (res == 0 && val != regVal) {
            INV_DEBUG("imu  rw error");
            res |= -1;
        }
    }
    return res;
}
int IMU_ReadReg(inv_imu_handle this, uint8_t reg, uint8_t *val) {
    int res = 0;
    if (this->i2c != NULL) {
        this->i2cTransfer.subAddress = reg;
        this->i2cTransfer.data = val;
        this->i2cTransfer.dataSize = 1;
        this->i2cTransfer.direction = inv_i2c_direction_Read;
        res = this->i2c->masterTransferBlocking(&this->i2cTransfer);
        if (res != 0) {
            INV_DEBUG("i2c read return code = %d", res);
        }
    } else {
        uint8_t txb[2];
        uint8_t rxb[2];
        txb[0] = (1U << 7U) | (reg & 0x7f);
        this->spiTransfer.dataSize = 2;
        this->spiTransfer.rxData = rxb;
        this->spiTransfer.txData = txb;
        res = this->spi->masterTransferBlocking(&this->spiTransfer);
        if (res != 0) {
            INV_DEBUG("spi read return code = %d", res);
        } else {
            *val = rxb[1];
        }

    }
    return res;
}
int IMU_ModifyReg(inv_imu_handle this, uint8_t reg, uint8_t val, uint8_t mask) {
    uint8_t regVal;
    int res = 0;
    res |= IMU_ReadReg(this, reg, &regVal);
    res |= IMU_WriteRegVerified(this, reg, (regVal & (~mask)) | (val & mask));
    res |= IMU_ReadReg(this, reg, &regVal);
    if ((regVal & mask) != (val & mask)) {
        INV_DEBUG("imu rw error");
        res |= -1;
    }
    return res;
}
