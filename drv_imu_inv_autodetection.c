#include "drv_imu_invensense.h"
#include "drv_imu_inv_mpu6050.h"
#include "drv_imu_inv_mpu9250.h"
#include "drv_imu_inv_icm20602.h"
#include "drv_imu_inv_icm20600.h"
#include "drv_imu_inv_icm20948.h"



#define IMU_AutoConstructI2CItem(model) {rtv = model##_ConstructI2C(_i2c,_addr);\
if(IMU_Detect(rtv)){\
return rtv;\
}else{\
IMU_Destruct(rtv);\
};}

inv_imu_handle IMU_AutoConstructI2C(inv_i2c _i2c, uint8_t _addr) {
    void *rtv = NULL;
#if defined(INV_MPU6050_ENABLE)&&(INV_MPU6050_ENABLE>0U)
    IMU_AutoConstructI2CItem(MPU6050);
#endif
#if defined(INV_MPU9250_ENABLE)&&(INV_MPU9250_ENABLE>0U)
    IMU_AutoConstructI2CItem(MPU9250);
#endif
#if defined(INV_ICM20602_ENABLE)&&(INV_ICM20602_ENABLE>0U)
    IMU_AutoConstructI2CItem(ICM20602);
#endif
#if defined(INV_ICM20600_ENABLE)&&(INV_ICM20600_ENABLE>0U)
    IMU_AutoConstructI2CItem(ICM20600);
#endif
#if defined(INV_ICM20948_ENABLE)&&(INV_ICM20948_ENABLE>0U)
    IMU_AutoConstructI2CItem(ICM20948);
#endif
    return rtv;
}

#define IMU_AutoConstructSPIItem(model) {rtv = model##_ConstructSPI(_spi);\
if(IMU_Detect(rtv)){\
return rtv;\
}else{\
IMU_Destruct(rtv);\
};}



inv_imu_handle IMU_AutoConstructSPI(inv_spi _spi) {
    void *rtv = NULL;
#if defined(INV_MPU9250_ENABLE)&&(INV_MPU9250_ENABLE>0U)
    IMU_AutoConstructSPIItem(MPU9250);
#endif
#if defined(INV_ICM20602_ENABLE)&&(INV_ICM20602_ENABLE>0U)
    IMU_AutoConstructSPIItem(ICM20602);
#endif
#if defined(INV_ICM20600_ENABLE)&&(INV_ICM20600_ENABLE>0U)
    IMU_AutoConstructSPIItem(ICM20600);
#endif
#if defined(INV_ICM20948_ENABLE)&&(INV_ICM20948_ENABLE>0U)
    IMU_AutoConstructSPIItem(ICM20948);
#endif
    return rtv;
}
