/**
 * @brief 陀螺仪驱动，适用于mpu6050,mpu9250,icm20602
 * @author xiao qq1761690868
 * @doc drv_imu_invensense.md
 * @version v1.0
 * @date 2020-10-16
 */

#ifndef D_MK66F18_DRV_IMU_INVENSENSE_PORT_HPP
#define D_MK66F18_DRV_IMU_INVENSENSE_PORT_HPP


#include "hitsic_common.h"

#if (defined(HITSIC_USE_DRV_IMU_INV) && (HITSIC_USE_DRV_IMU_INV > 0U))
//切换对模块的支持
#define INV_MPU6050_ENABLE 1
#define INV_MPU9250_ENABLE 1
#define INV_ICM20602_ENABLE 1
#define INV_ICM20600_ENABLE 1
#define INV_ICM20948_ENABLE 1

//切换 trace和debug
#define INV_YES_TRACE 1
#define INV_NO_DEBUG 0

//设置使用什么堆内存管理
#define INV_MALLOC malloc
#define INV_FREE free
#define INV_REALLOC realloc

// 设置printf
#if !defined(INV_PRINTF)
#include<stdio.h>
#define INV_PRINTF printf
#endif //!defined(INV_PRINTF)

#if INV_YES_TRACE
#define INV_TRACE_(fmt, ...) \
    INV_PRINTF("%s:%d:trace: " fmt "%s\r\n", __FILE__, __LINE__, __VA_ARGS__)
#define INV_TRACE(...) INV_TRACE_(__VA_ARGS__, "")
#else
#define INV_TRACE(...)
#endif //INV_YES_TRACE

#if !INV_NO_DEBUG
#define INV_DEBUG_(fmt, ...) \
    INV_PRINTF("%s:%d:debug: " fmt "%s\r\n", __FILE__, __LINE__, __VA_ARGS__)
#define INV_DEBUG(...) INV_DEBUG_(__VA_ARGS__, "")
#else
#define INV_DEBUG(...)
#endif //INV_NO_DEBUG


//#if defined(__cplusplus) || defined(c_plusplus)
//extern "C"{
//#endif

typedef enum __inv_i2c_direction {
    inv_i2c_direction_Write = 0U, /*!< Master transmit. */
    inv_i2c_direction_Read = 1U  /*!< Master receive. */
} inv_i2c_direction;

typedef struct __inv_i2c_transfer {
    uint16_t slaveAddress;
    uint8_t slaveAddressSize;
    uint8_t subAddressSize;
    uint32_t subAddress;
    void *data;
    uint32_t dataSize;
    inv_i2c_direction direction;
} inv_i2c_transfer;

typedef struct __inv_i2c {
    int (*masterTransferBlocking)(const inv_i2c_transfer *);
    int (*masterTransferNonBlocking)(const inv_i2c_transfer *);
} inv_i2c;


typedef struct __inv_spi_transfer {
    uint8_t *txData;          /*!< Send buffer. */
    uint8_t *rxData;          /*!< Receive buffer. */
    volatile uint32_t dataSize; /*!< Transfer bytes. */
} inv_spi_transfer;
typedef struct __inv_spi {
    int (*masterTransferBlocking)(const inv_spi_transfer *);
    int (*masterTransferNonBlocking)(const inv_spi_transfer *);
} inv_spi;

inline void IMU_DelayUs(uint32_t delay_ms)
{
    SDK_DelayAtLeastUs(delay_ms,CLOCK_GetFreq(kCLOCK_CoreSysClk));
}

inline int IMU_I2C0_MasterXferBlocking(const inv_i2c_transfer *xfer)
{
    static i2c_master_transfer_t i2c_xfer;
        i2c_xfer.slaveAddress = xfer->slaveAddress;
        i2c_xfer.direction = (i2c_direction_t)xfer->direction;
        i2c_xfer.subaddress = xfer->subAddress;
        i2c_xfer.subaddressSize = xfer->subAddressSize;
        i2c_xfer.data = xfer->data;
        i2c_xfer.dataSize = xfer->dataSize;
        i2c_xfer.flags = kI2C_TransferDefaultFlag;
    return I2C_MasterTransferBlocking(I2C0, &i2c_xfer);
}

#endif // ! HITSIC_USE_DRV_IMU_INV

#endif //! D_MK66F18_DRV_IMU_INVENSENSE_PORT_HPP
