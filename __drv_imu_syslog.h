#ifndef C_INV_IMU____DRV_IMU_SYSLOG_H
#define C_INV_IMU____DRV_IMU_SYSLOG_H
#include <drv_imu_invensense_port.h>

#if (defined(INV_USE_HITSIC_SYSLOG) && (INV_USE_HITSIC_SYSLOG > 0))

#define SYSLOG_LVL (INVIMU_LOG_LVL)
#define SYSLOG_TAG "INVIMU"
#include<inc_syslog.h>

#else // INV_USE_HITSIC_SYSLOG

#define SYSLOG_A(...) INV_PRINTF(__VA_ARGS__);INV_PRINTF("\r\n")
#define SYSLOG_E(...) INV_PRINTF(__VA_ARGS__);INV_PRINTF("\r\n")
#define SYSLOG_W(...) INV_PRINTF(__VA_ARGS__);INV_PRINTF("\r\n")
#define SYSLOG_I(...) INV_PRINTF(__VA_ARGS__);INV_PRINTF("\r\n")
#define SYSLOG_D(...) INV_PRINTF(__VA_ARGS__);INV_PRINTF("\r\n")
#define SYSLOG_V(...) INV_PRINTF(__VA_ARGS__);INV_PRINTF("\r\n")

#endif // ! INV_USE_HITSIC_SYSLOG


#endif //C_INV_IMU____DRV_IMU_SYSLOG_H
