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


static void *(*const ConstructI2C_table[])(inv_i2c_t, uint8_t) ={
        (void *(*)(inv_i2c_t, uint8_t)) MPU6050_ConstructI2C,
        (void *(*)(inv_i2c_t, uint8_t)) MPU9250_ConstructI2C,
        (void *(*)(inv_i2c_t, uint8_t)) ICM20602_ConstructI2C,
        (void *(*)(inv_i2c_t, uint8_t)) ICM20600_ConstructI2C,
        (void *(*)(inv_i2c_t, uint8_t)) ICM20948_ConstructI2C
};

inv_imu_handle_t IMU_AutoConstructI2C(inv_i2c_t _i2c, uint8_t _addr) {
    void *rtv = NULL;
    for (int i = 0; i < sizeof(ConstructI2C_table) / sizeof(void *(*)(inv_i2c_t, uint8_t)); ++i) {
        rtv = ConstructI2C_table[i](_i2c, _addr);
        assert(rtv);
        if (IMU_Detect(rtv)) {
            return rtv;
        } else {
            IMU_Destruct(rtv);
            rtv = NULL;
        };
    }
    return rtv;
}

static void *(*const ConstructSPI_table[])(inv_spi_t) ={
        (void *(*)(inv_spi_t)) MPU9250_ConstructSPI,
        (void *(*)(inv_spi_t)) ICM20602_ConstructSPI,
        (void *(*)(inv_spi_t)) ICM20600_ConstructSPI,
        (void *(*)(inv_spi_t)) ICM20948_ConstructSPI
};

inv_imu_handle_t IMU_AutoConstructSPI(inv_spi_t _spi) {
    void *rtv = NULL;
    for (int i = 0; i < sizeof(ConstructSPI_table) / sizeof(void *(*)(inv_spi_t)); ++i) {
        rtv = ConstructSPI_table[i](_spi);
        assert(rtv);
        if (IMU_Detect(rtv)) {
            return rtv;
        } else {
            IMU_Destruct(rtv);
            rtv = NULL;
        };
    }
    return rtv;
}
