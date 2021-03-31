# c_inv_imu
[TOC]
## 简介
修改自c++版本驱动，保留原汁原味的c++体验


## 传输

### i2c

接口如下，需要自行实现读写i2c并构造inv_i2c，注意其中-Blocking和-NonBlocking和imu驱动中的IMU_ReadSensorBlocking和IMU_ReadSensorNonBlocking对应

```c
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
```

### spi

大体和i2c一致，但spi使用时是全双工传输

```c
typedef struct __inv_spi_transfer {
    uint8_t *txData;          /*!< Send buffer. */
    uint8_t *rxData;          /*!< Receive buffer. */
    volatile uint32_t dataSize; /*!< Transfer bytes. */
} inv_spi_transfer;
typedef struct __inv_spi {
    int (*masterTransferBlocking)(const inv_spi_transfer *);
    int (*masterTransferNonBlocking)(const inv_spi_transfer *);
} inv_spi;

```

## 如何使用



```c
#include <stdion.h>
#include "inv_imu.h"
int main_test(int argn,char** argv){
    
    inv_i2c myi2c;
    //myi2c = ....
    //or
    //inv_spi myspi;
    //myspi=.....
    inv_imu_handle imu0 = IMU_AutoConstruct(myi2c,IMU_SlaveAddressAutoDetect);
    
    if(imu0!=NULL){
        printf("当前使用的传感器为 %s\r\n",IMU_Report(imu0));

        if(0==IMU_Init(imu0,IMU_ConfigDefault())){
            printf("ST = %d,0为自检通过,自检时保持静止\r\n",IMU_SelfTest(imu0));
            Delay(10)//delay 10ms

            
            int loop = 10;
            while(loop--){
                IMU_ReadSensorBlocking(imu0);
                float temp;
                float data[9];
                IMU_Convert(imu0, data);
                IMU_Convert(&temp);
                
                float* buf = data;
                printf("temp = %f",temp);
                printf("accel(xyz) = %.3f %.3f %.3f\t gyro(xyz) = %.3f %.3f %.3f\t mag(xyz) = %.3f %.3f %.3f",
                       *buf++,
                       *buf++,
                       *buf++,

                        *buf++,
                        *buf++,
                        *buf++,
                        
                        *buf++,
                        *buf++,
                        *buf++
                );
            }
            
        }
        printf("初始化失败\r\n");
    }
    else{
        printf("imu0 == NULL，没接或者iic/spi读写出错\r\n");
    }
    if(imu0!=NULL){
        IMU_Destruct(imu0);
    }
    return 0;
    
}





```



### 半自动

参考inv_imu_fire.c中IMU_AutoConstruct的实现



### 精准

```c
#include <stdion.h>
#include "inv_imu.h"
#include "inv_mpu9250.h"
int main_test(int argn,char** argv){
    
    inv_i2c myi2c;
    //myi2c = ....
    //or
    //inv_spi myspi;
    //myspi=.....
    inv_imu_handle imu0 = MPU9250_Construct(myi2c,IMU_SlaveAddressAutoDetect);
    
    if(imu0!=NULL){
        printf("当前使用的传感器为 MPU9250");

        if(0==IMU_Init(imu0,IMU_ConfigDefault())){
            printf("ST = %d,0为自检通过,自检时保持静止\r\n",IMU_SelfTest(imu0));
            Delay(10)//delay 10ms

            
            int loop = 10;
            while(loop--){
                IMU_ReadSensorBlocking(imu0);
                float temp;
                float data[9];
                IMU_Convert(imu0, data);
                IMU_Convert(&temp);
                
                float* buf = data;
                printf("temp = %f",temp);
                printf("accel(xyz) = %.3f %.3f %.3f\t gyro(xyz) = %.3f %.3f %.3f\t mag(xyz) = %.3f %.3f %.3f",
                       *buf++,
                       *buf++,
                       *buf++,

                        *buf++,
                        *buf++,
                        *buf++,
                        
                        *buf++,
                        *buf++,
                        *buf++
                );
            }
            
        }
        printf("初始化失败\r\n");
    }
    else{
        printf("imu0 == NULL，没接MPU9250或者iic/spi读写出错\r\n");
    }
    if(imu0!=NULL){
        IMU_Destruct(imu0);
    }
    return 0;
    
}





```

### 注意事项

不能这样使用

```c
inv_imu imu0;
IMU_Init(&imu0,IMU_ConfigDefault());
```

## LICENSE

