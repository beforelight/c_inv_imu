# c_inv_imu
[TOC]
## 简介
修改自c++版本驱动，保留原汁原味的c++体验

## 使用

### 移植

拷貝drv_imu_invensense_port_template.h到**头文件路径**下，并重命名为drv_imu_invensense_port.h，之后根据提示修改相关宏开关相关功能。

drv_imu_invensense_port.h中定义了如下传输过程控制结构体，移植时可以在”you like"处添加自定义的成员，比如说，代表使用第几个i2c的变量。

```c
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

typedef struct __inv_spi_transfer {
    uint8_t *volatile txData;          /*!< Send buffer. */
    uint8_t *volatile rxData;          /*!< Receive buffer. */
    volatile uint32_t dataSize; /*!< Transfer bytes. */
    /*******************************************************/
    //you like
} inv_spi_transfer_t;
```

drv_imu_invensense_port.h中定义了如下I2C,SPI传输接口，在构造imu对象时需要传递此结构体的变量。**注意：SPI使用的是全双工传输** 。此外API——xxx_ReadSensorBlocking调用的接口为masterTransferBlocking，API——xxx_ReadSensorNonBlocking调用的接口为masterTransferNonBlocking。**移植至少需实现其‘masterTransferBlocking’**

```c
typedef struct __inv_i2c {
    int (*masterTransferBlocking)(const inv_i2c_transfer_t *);
    int (*masterTransferNonBlocking)(const inv_i2c_transfer_t *);
} inv_i2c_t;

typedef struct __inv_spi {
    int (*masterTransferBlocking)(const inv_spi_transfer_t *);
    int (*masterTransferNonBlocking)(const inv_spi_transfer_t *);
} inv_spi_t;
```



### 示例

见example



## API

> 以下说明中，xxx或者yyy可以是IMU，MPU6050，ICM20602......等等模块名字
>
> 具体api以代码为准，文档可能不会及时更新
>
> **如果某模块不支持一些api，则会直接返回不影响代码执行的结果**

因为是从c++代码翻译而来，c代码版本中对于一个xxx来说，inv_xxx_t为类，inv_xxx_handle_t为类指针，xxx_开头的函数为其成员函数，第一个传参总是inv_xxx_handle_t类型。

若成员中出现了inv_yyy_t parents；说明inv_xxx_t继承了inv_yyy_t 。inv_xxx_t的对象可以使用inv_yyy_t 的成员的函数(c语言版本的需要使用类型强转)，反之则不行。

**inv_imu_t为本驱动的虚基类。** 

如果上面没说清的话，看看示例。。。。。。。。



下面是api的简单说明

```c
//获取默认的配置
inv_imu_config_t IMU_ConfigDefault(); 

//初始化，返回错误代码(0=无错误)
int xxx_Init(inv_imu_handle_t _this, inv_imu_config_t _cfg) ;

//检查模块连接与否，无需经过初始化
bool xxx_Detect(inv_imu_handle_t _this) ;

//自检，自检时请保持精子，返回0为自检通过
int xxx_SelfTest(inv_imu_handle_t _this) ;

//返回模块名称，比如“mpu6050"
const char *xxx_Report(inv_imu_handle_t _this) ;

//检查模块是否获取了新的一帧数据
bool xxx_DataReady(inv_imu_handle_t _this) ;

//打开DataReady中断，返回错误代码(0=无错误)
int xxx_EnableDataReadyInt(inv_imu_handle_t _this) ;

//复位，返回错误代码(0=无错误)
int xxx_SoftReset(inv_imu_handle_t _this) ;

//使用阻塞读取(参考inv_spi_t\inv_i2c_t)，将原始数据从模块读取到缓存，返回错误代码(0=无错误)
int xxx_ReadSensorBlocking(inv_imu_handle_t _this) ;

//使用非阻塞读取(参考inv_spi_t\inv_i2c_t)，将原始数据从模块读取到缓存，返回错误代码(0=无错误)
int xxx_ReadSensorNonBlocking(inv_imu_handle_t _this) ;

//从缓存中转换单位到array,顺序是 accel(xyz) gyro(xyz) mag(xyz)，返回错误代码(0=无错误)
int xxx_Convert(inv_imu_handle_t _this, float array[9]) ;
//拷贝缓存到raw,顺序是 accel(xyz) gyro(xyz) mag(xyz)，返回错误代码(0=无错误)
int xxx_ConvertRaw(inv_imu_handle_t _this, int16_t raw[9]) ;
//转换缓存中的数据到温度，，返回错误代码(0=无错误)
int xxx_ConvertTemp(inv_imu_handle_t _this, float *temp) ;

//检查初始化与否
bool xxx_IsOpen(inv_imu_handle_t _this) ;

//析构，释放资源(约等于free)
void xxx_Destruct(inv_imu_handle_t _this) ;

//malloc内存后构造以I2C为接口的xxx
inv_xxx_handle_t xxx_ConstructI2C(inv_i2c_t _i2c, uint8_t _addr);
//malloc内存后构造以SPI为接口的xxx
inv_xxx_handle_t xxx_ConstructSPI(inv_spi_t _spi);
```



### 注意事项

不能像下面这样使用，必须使用构造函数构造。

```c
inv_imu imu0;
IMU_Init(&imu0,IMU_ConfigDefault());
```

## LICENSE

