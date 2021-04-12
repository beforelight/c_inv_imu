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
#include<stdio.h>

#define INV_ASSERT(...) printf("%s:%d assert:",__FILE__, __LINE__);printf(__VA_ARGS__);printf("\r\n")
#define INV_ERROR(...)  printf("%s:%d error:",__FILE__, __LINE__);printf(__VA_ARGS__);printf("\r\n")
#define INV_INFO(...)   printf("%s:%d info:",__FILE__, __LINE__);printf(__VA_ARGS__);printf("\r\n")
#define INV_DEBUG(...)  printf("%s:%d debug:",__FILE__, __LINE__);printf(__VA_ARGS__);printf("\r\n")
#endif


#include<stdint.h>

typedef enum __inv_i2c_direction {
    inv_i2c_direction_Write = 0U, /*!< Master transmit. */
    inv_i2c_direction_Read = 1U  /*!< Master receive. */
} inv_i2c_direction;

typedef struct __inv_i2c_transfer {
    uint8_t slaveAddress;      /*!< 7-bit slave address. */
    uint8_t subAddressSize;     /*!< A size of the command buffer. */
    uint32_t subAddress;        /*!< A sub address. Transferred MSB first. */
    void *volatile data;        /*!< A transfer buffer. */
    volatile uint32_t dataSize;          /*!< A transfer size. */
    inv_i2c_direction direction; /*!< A transfer direction, read or write. */
    /*******************************************************/
    //you like
} inv_i2c_transfer;

typedef struct __inv_i2c {
    int (*masterTransferBlocking)(const inv_i2c_transfer *);
    int (*masterTransferNonBlocking)(const inv_i2c_transfer *);
} inv_i2c;

typedef struct __inv_spi_transfer {
    uint8_t *volatile txData;          /*!< Send buffer. */
    uint8_t *volatile rxData;          /*!< Receive buffer. */
    volatile uint32_t dataSize; /*!< Transfer bytes. */
    /*******************************************************/
    //you like
} inv_spi_transfer;

typedef struct __inv_spi {
    int (*masterTransferBlocking)(const inv_spi_transfer *);
    int (*masterTransferNonBlocking)(const inv_spi_transfer *);
} inv_spi;

#endif //INV_IMU_INV_CONF_H
