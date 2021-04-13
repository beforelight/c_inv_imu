#ifndef INV_IMU_INV_CONF_H
#define INV_IMU_INV_CONF_H
#if 1
//切换对模块的支持
#define INV_MPU6050_ENABLE 1
#define INV_MPU9250_ENABLE 1
#define INV_ICM20602_ENABLE 1
#define INV_ICM20600_ENABLE 1
#define INV_ICM20948_ENABLE 1

//设置使用什么堆内存管理
#include<stdlib.h>

#define INV_MALLOC malloc
#define INV_FREE free
#define INV_REALLOC realloc

//设置log接口，注意接口函数是带换行符的printf

#define INV_USE_HITSIC_SYSLOG (0U)

#if (defined(INV_USE_HITSIC_SYSLOG) && (INV_USE_HITSIC_SYSLOG > 0))

#define INVIMU_LOG_LVL (3U)

#else // INV_USE_HITSIC_SYSLOG

#include <stdio.h>
#define INV_PRINTF(...) printf(__VA_ARGS__);printf("\r\n")

#endif // ! INV_USE_HITSIC_SYSLOG

#endif // ! 1


#include<stdint.h>

typedef enum __inv_i2c_direction {
    inv_i2c_direction_Write = 0U, /*!< Master transmit. */
    inv_i2c_direction_Read = 1U  /*!< Master receive. */
} inv_i2c_direction_t;

typedef struct __inv_i2c_transfer {
    uint8_t slaveAddress;      /*!< 7-bit slave address. */
    uint8_t subAddressSize;     /*!< A size of the command buffer. */
    uint32_t subAddress;        /*!< A sub address. Transferred MSB first. */
    void *volatile data;        /*!< A transfer buffer. */
    volatile uint32_t dataSize;          /*!< A transfer size. */
    inv_i2c_direction_t direction; /*!< A transfer direction, read or write. */
    /*******************************************************/
    //you like
} inv_i2c_transfer_t;

typedef struct __inv_i2c {
    int (*masterTransferBlocking)(const inv_i2c_transfer_t *);
    int (*masterTransferNonBlocking)(const inv_i2c_transfer_t *);
} inv_i2c_t;

typedef struct __inv_spi_transfer {
    uint8_t *volatile txData;          /*!< Send buffer. */
    uint8_t *volatile rxData;          /*!< Receive buffer. */
    volatile uint32_t dataSize; /*!< Transfer bytes. */
    /*******************************************************/
    //you like
} inv_spi_transfer_t;

typedef struct __inv_spi {
    int (*masterTransferBlocking)(const inv_spi_transfer_t *);
    int (*masterTransferNonBlocking)(const inv_spi_transfer_t *);
} inv_spi_t;

#endif //INV_IMU_INV_CONF_H
