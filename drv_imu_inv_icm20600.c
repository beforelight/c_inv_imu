#include "drv_imu_inv_icm20600.h"
#if defined(INV_ICM20600_ENABLE)&&(INV_ICM20600_ENABLE>0U)

//#if defined(__cplusplus) || defined(c_plusplus)
//extern "C"{
//#endif

const inv_imu_vector_table icm20600_VectorTable =
        {
                .Init = (void *) ICM20602_Init,
                .Detect =(void *) ICM20600_Detect,
                .SelfTest =(void *) ICM20602_SelfTest,
                .Report =(void *) ICM20600_Report,
                .DataReady =(void *) ICM20602_DataReady,
                .EnableDataReadyInt =(void *) ICM20602_EnableDataReadyInt,
                .SoftReset =(void *) ICM20602_SoftReset,
                .ReadSensorBlocking =(void *) ICM20602_ReadSensorBlocking,
                .ReadSensorNonBlocking =(void *) ICM20602_ReadSensorNonBlocking,
                .Convert =(void *) ICM20602_Convert,
                .ConvertRaw =(void *) ICM20602_ConvertRaw,
                .ConvertTemp =(void *) ICM20602_ConvertTemp,
                .IsOpen =(void *) _IMU_IsOpen,
                .Destruct = (void*) ICM20600_Destruct,
        };

inv_icm20600_handle ICM20600_ConstructI2C(inv_i2c _i2c, uint8_t _addr) {
    inv_icm20600_handle rtv = (inv_icm20600_handle) ICM20602_ConstructI2C(_i2c, _addr);
    rtv->parents.parents.vtable = &icm20600_VectorTable;
    return rtv;
}
inv_icm20600_handle ICM20600_ConstructSPI(inv_spi _spi) {
    inv_icm20600_handle rtv = (inv_icm20600_handle) ICM20602_ConstructSPI(_spi);
    rtv->parents.parents.vtable = &icm20600_VectorTable;
    return rtv;
}
bool ICM20600_Detect(inv_icm20602_handle _this) {
    uint8_t val = 0;
    if (_this->parents.addrAutoDetect) { _this->parents.i2cTransfer.slaveAddress = 0x68; }
    IMU_ReadReg((inv_imu_handle) _this, (uint8_t) ICM20602_WHO_AM_I, &val);
    if (0x11 == val) {
        return true;
    }
    val = 0;
    if (_this->parents.addrAutoDetect) { _this->parents.i2cTransfer.slaveAddress = 0x69; }
    IMU_ReadReg((inv_imu_handle) _this, (uint8_t) ICM20602_WHO_AM_I, &val);
    if (0x11 == val) {
        return true;
    }
    return false;
}

//#if defined(__cplusplus) || defined(c_plusplus)
//}
//#endif

#endif //INV_XXX_ENABLE

