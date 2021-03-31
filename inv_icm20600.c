#if defined(__cplusplus) || defined(c_plusplus)
extern "C"{
#endif

#include "inv_icm20600.h"
#if INV_ICM20600_ENABLE
inv_imu_vector_table icm20600_VectorTable =
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
                .Convert2 =(void *) ICM20602_Convert2,
                .Convert3 =(void *) ICM20602_Convert3,
                .IsOpen =(void *) _IMU_IsOpen,
                .Destruct = (void*) ICM20600_Destruct
        };

inv_icm20600_handle ICM20600_Construct(inv_i2c _i2c, uint16_t _addr) {
    inv_icm20600_handle rtv = (inv_icm20600_handle) ICM20602_Construct(_i2c, _addr);
    rtv->parents.parents.vtable = &icm20600_VectorTable;
    return rtv;
}
inv_icm20600_handle ICM20600_Construct2(inv_spi _spi) {
    inv_icm20600_handle rtv = (inv_icm20600_handle) ICM20602_Construct2(_spi);
    rtv->parents.parents.vtable = &icm20600_VectorTable;
    return rtv;
}
bool ICM20600_Detect(inv_icm20602_handle this) {
    uint8_t val = 0;
    if (this->parents.addrAutoDetect) { this->parents.i2cTransfer.slaveAddress = 0x68; }
    IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_WHO_AM_I, &val);
    if (0x11 == val) {
        return true;
    }
    val = 0;
    if (this->parents.addrAutoDetect) { this->parents.i2cTransfer.slaveAddress = 0x69; }
    IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_WHO_AM_I, &val);
    if (0x11 == val) {
        return true;
    }
    return false;
}

#endif //INV_XXX_ENABLE
#if defined(__cplusplus) || defined(c_plusplus)
}
#endif