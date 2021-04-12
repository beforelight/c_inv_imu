#include "drv_imu_invensense.h"
#include "drv_imu_inv_mpu6050.h"
#include "drv_imu_inv_mpu9250.h"
#include "drv_imu_inv_icm20602.h"
#include "drv_imu_inv_icm20600.h"
#include "drv_imu_inv_icm20948.h"



#define IMU_AutoConstructItem(model) {rtv = model##_Construct(_i2c,_addr);\
if(IMU_Detect(rtv)){\
return rtv;\
}else{\
IMU_Destruct(rtv);\
};}

inv_imu_handle IMU_AutoConstruct(inv_i2c _i2c, uint8_t _addr) {
    void *rtv = NULL;
#if INV_MPU6050_ENABLE
    IMU_AutoConstructItem(MPU6050);
#endif
#if INV_MPU9250_ENABLE
    IMU_AutoConstructItem(MPU9250);
#endif
#if INV_ICM20602_ENABLE
    IMU_AutoConstructItem(ICM20602);
#endif
#if INV_ICM20600_ENABLE
    IMU_AutoConstructItem(ICM20600);
#endif
#if INV_ICM20948_ENABLE
    IMU_AutoConstructItem(ICM20948);
#endif
    return rtv;
}

#define IMU_AutoConstruct2Item(model) {rtv = model##_Construct2(_spi);\
if(IMU_Detect(rtv)){\
return rtv;\
}else{\
IMU_Destruct(rtv);\
};}



inv_imu_handle IMU_AutoConstruct2(inv_spi _spi) {
    void *rtv = NULL;
#if INV_MPU9250_ENABLE
    IMU_AutoConstruct2Item(MPU9250);
#endif
#if INV_ICM20602_ENABLE
    IMU_AutoConstruct2Item(ICM20602);
#endif
#if INV_ICM20600_ENABLE
    IMU_AutoConstruct2Item(ICM20600);
#endif
#if INV_ICM20948_ENABLE
    IMU_AutoConstruct2Item(ICM20948);
#endif
    return rtv;
}
