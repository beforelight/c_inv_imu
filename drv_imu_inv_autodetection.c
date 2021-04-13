#include "drv_imu_invensense.h"
#include "drv_imu_inv_mpu6050.h"
#include "drv_imu_inv_mpu9250.h"
#include "drv_imu_inv_icm20602.h"
#include "drv_imu_inv_icm20600.h"
#include "drv_imu_inv_icm20948.h"

#if (defined(INV_USE_HITSIC_SYSLOG) && (INV_USE_HITSIC_SYSLOG > 0))

#define SYSLOG_LVL (INVIMU_LOG_LVL)
#define SYSLOG_TAG "INVIMU"
#include<inc_syslog.h>

#else // INV_USE_HITSIC_SYSLOG

#define SYSLOG_A(...) INV_PRINTF("\r\n");
#define SYSLOG_E(...) INV_PRINTF("\r\n");
#define SYSLOG_W(...) INV_PRINTF("\r\n");
#define SYSLOG_I(...) INV_PRINTF("\r\n");
#define SYSLOG_D(...) INV_PRINTF("\r\n");
#define SYSLOG_V(...) INV_PRINTF("\r\n");

#endif // ! INV_USE_HITSIC_SYSLOG



#define IMU_AutoConstructI2CItem(model) {rtv = model##_ConstructI2C(_i2c,_addr);\
assert(rtv);\
if(IMU_Detect(rtv)){\
return rtv;\
}else{\
IMU_Destruct(rtv);\
};}

inv_imu_handle_t IMU_AutoConstructI2C(inv_i2c_t _i2c, uint8_t _addr) {
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
assert(rtv);\
if(IMU_Detect(rtv)){\
return rtv;\
}else{\
IMU_Destruct(rtv);\
};}



inv_imu_handle_t IMU_AutoConstructSPI(inv_spi_t _spi) {
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
