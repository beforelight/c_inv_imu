#include "drv_imu_inv_mpu6050.h"
#if INV_MPU6050_ENABLE

//#if defined(__cplusplus) || defined(c_plusplus)
//extern "C"{
//#endif

const uint16_t accelSelfTestEquation[32] = {
        1347, 1393, 1440, 1488, 1538, 1590, 1644, 1699,
        1757, 1816, 1877, 1941, 2006, 2074, 2144, 2216,
        2291, 2368, 2448, 2531, 2616, 2704, 2795, 2890,
        2987, 3088, 3192, 3300, 3411, 3526, 3645, 3768,
};


const uint16_t gyroSelfTestEquation[32] = {
        3131, 3275, 3426, 3583, 3748, 3920, 4101, 4289,
        4487, 4693, 4909, 5135, 5371, 5618, 5877, 6147,
        6430, 6725, 7035, 7358, 7697, 8051, 8421, 8809,
        9214, 9638, 10081, 10545, 11030, 11537, 12068, 12623};


inv_imu_vector_table mpu6050_VectorTable = {
        .Init = (void *) MPU6050_Init,
        .Detect =(void *) MPU6050_Detect,
        .SelfTest =(void *) MPU6050_SelfTest,
        .Report =(void *) MPU6050_Report,
        .DataReady =(void *) MPU6050_DataReady,
        .EnableDataReadyInt =(void *) MPU6050_EnableDataReadyInt,
        .SoftReset =(void *) MPU6050_SoftReset,
        .ReadSensorBlocking =(void *) MPU6050_ReadSensorBlocking,
        .ReadSensorNonBlocking =(void *) MPU6050_ReadSensorNonBlocking,
        .Convert =(void *) MPU6050_Convert,
        .Convert2 =(void *) MPU6050_Convert2,
        .Convert3 =(void *) MPU6050_Convert3,
        .IsOpen =(void *) _IMU_IsOpen,
        .Destruct = (void*) MPU6050_Destruct
};

inv_mpu6050_handle MPU6050_Construct(inv_i2c _i2c, uint16_t _addr) {
    inv_mpu6050_handle rtv = (void *) INV_REALLOC(IMU_Construct(_i2c, _addr), sizeof(inv_mpu6050));
    memset((void *) ((char *)rtv + sizeof(inv_mpu6050) - sizeof(inv_imu)), 0, sizeof(inv_mpu6050) - sizeof(inv_imu));
    rtv->parents.vtable = &mpu6050_VectorTable;
    return rtv;
}

int MPU6050_Init(inv_mpu6050_handle _this, inv_imu_config _cfg) {
    _this->parents.cfg = _cfg;
    _this->parents.isOpen = false;
    int res = 0;
    int bw;
    int fs;
    float unit;
    float unit_from;


    if (!IMU_Detect((inv_imu_handle)_this)) { return -1; }
    //软复位
    res |= IMU_SoftReset((inv_imu_handle)_this);

    //打开所有传感器
    res |= IMU_WriteRegVerified((inv_imu_handle)_this, (uint8_t) MPU6050_PWR_MGMT_2, 0);

    //1khz采样率
    res |= IMU_WriteRegVerified((inv_imu_handle)_this, (uint8_t) MPU6050_SMPLRT_DIV, 0);

    //配置陀螺仪lpf
    _InvGetMapVal(MPU6050_GBW_MAP, _this->parents.cfg.gyroBandwidth, bw);
    res |= IMU_WriteRegVerified((inv_imu_handle)_this, (uint8_t) MPU6050_CONFIG, bw);

    //配置陀螺仪量程和单位
    _InvGetMapVal(mpu_gyro_unit_dps_map, _this->parents.cfg.gyroFullScale, unit);
    _InvGetMapVal(mpu_gyro_unit_from_dps_map, _this->parents.cfg.gyroUnit, unit_from);
    _this->gyroUnit = unit * unit_from;

    _InvGetMapVal(mpu_gyro_fs_map, _this->parents.cfg.gyroFullScale, fs);
    res |= IMU_WriteRegVerified((inv_imu_handle)_this, (uint8_t) MPU6050_GYRO_CONFIG, fs << 3u);

    //配置加速度计量程和单位
    _InvGetMapVal(mpu_accel_unit_G_map, _this->parents.cfg.accelFullScale, unit);
    _InvGetMapVal(mpu_accel_unit_from_G_map, _this->parents.cfg.accelUnit, unit_from);
    _this->accelUnit = unit * unit_from;

    _InvGetMapVal(mpu_accel_fs_map, _this->parents.cfg.accelFullScale, fs);
    res |= IMU_WriteRegVerified((inv_imu_handle)_this, (uint8_t) MPU6050_ACCEL_CONFIG, fs << 3u);

    //开启数据更新中断
    res |= IMU_EnableDataReadyInt((inv_imu_handle)_this);

    if (res == 0) {
        _this->parents.isOpen = true;
    }
    return res;
}
bool MPU6050_Detect(inv_mpu6050_handle _this) {
    uint8_t val = 0;
    if (_this->parents.addrAutoDetect) { _this->parents.i2cTransfer.slaveAddress = 0x68; };
    IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_WHO_AM_I, &val);
    if (0x68 == val) {
        return true;
    }
    val = 0;
    if (_this->parents.addrAutoDetect) { _this->parents.i2cTransfer.slaveAddress = 0x69; };
    IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_WHO_AM_I, &val);
    if (0x68 == val) {
        return true;
    }
    return false;
}
int MPU6050_SelfTest(inv_mpu6050_handle _this) {
    if (!IMU_IsOpen((inv_imu_handle)_this)) { return -1; }
    int res = 0;
    inv_imu_config backup_cfg = _this->parents.cfg;
    inv_imu_config st_cfg = IMU_ConfigDefault();
    st_cfg.gyroFullScale = MPU_FS_250dps;
    st_cfg.accelFullScale = MPU_FS_8G;
    if (0 != IMU_Init((inv_imu_handle)_this, st_cfg)) {
        IMU_Init((inv_imu_handle)_this, backup_cfg);
        return -1;
    }
    int32_t gyro_bias_st[3], gyro_bias_regular[3];
    int32_t accel_bias_st[3], accel_bias_regular[3];
    int16_t abuf[9];
    int16_t *gbuf = &abuf[3];
    int accel_result = 0;
    int gyro_result = 0;
    uint8_t val;
    memset(gyro_bias_st, 0, sizeof(gyro_bias_st));
    memset(gyro_bias_regular, 0, sizeof(gyro_bias_regular));
    memset(accel_bias_st, 0, sizeof(accel_bias_st));
    memset(accel_bias_regular, 0, sizeof(accel_bias_regular));


    int times;
    times = 20;
    while (times--) { while (!IMU_DataReady((inv_imu_handle)_this)) {}}//丢弃前20个数据
    times = 20;
    while (times--) {
        while (!IMU_DataReady((inv_imu_handle)_this)) {}
        res |= IMU_ReadSensorBlocking((inv_imu_handle)_this);
        IMU_Convert2((inv_imu_handle)_this, abuf);
        for (int i = 0; i < 3; ++i) {
            gyro_bias_regular[i] += gbuf[i];
            accel_bias_regular[i] += abuf[i];
        }
    }

    res |= IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_GYRO_CONFIG, &val);
    res |= IMU_WriteRegVerified((inv_imu_handle)_this, (uint8_t) MPU6050_GYRO_CONFIG, val | (0b111 << 5));//打开陀螺仪自检
    res |= IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_ACCEL_CONFIG, &val);
    res |= IMU_WriteRegVerified((inv_imu_handle)_this, (uint8_t) MPU6050_ACCEL_CONFIG, val | (0b111 << 5));//打开加速度计自检
    times = 20;
    while (times--) { while (!IMU_DataReady((inv_imu_handle)_this)) {}}//丢弃前100个数据
    times = 20;
    while (times--) {
        while (!IMU_DataReady((inv_imu_handle)_this)) {}
        res |= IMU_ReadSensorBlocking((inv_imu_handle)_this);
        IMU_Convert2((inv_imu_handle)_this, abuf);
        for (int i = 0; i < 3; ++i) {
            gyro_bias_st[i] += gbuf[i];
            accel_bias_st[i] += abuf[i];
        }
    }

    for (int i = 0; i < 3; ++i) {
        gyro_bias_regular[i] *= 50;   //(32768/2000)*1000 LSB/mg
        accel_bias_regular[i] *= 50;
        gyro_bias_st[i] *= 50;         //(32768/250)*1000 LSB/dps
        accel_bias_st[i] *= 50;
    }

    //开始计算自检结果
    uint8_t regs[4];

    res |= IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_SELF_TEST_X, regs);
    res |= IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_SELF_TEST_Y, regs + 1);
    res |= IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_SELF_TEST_Z, regs + 2);
    res |= IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_SELF_TEST_A, regs + 3);
    int a_st[3];
    int g_st[3];
    int ft_a[3];
    int ft_g[3];
    a_st[0] = ((0b111 & (regs[0] >> 5)) << 2) | (0b11 & (regs[3] >> 4));
    a_st[1] = ((0b111 & (regs[1] >> 5)) << 2) | (0b11 & (regs[3] >> 2));
    a_st[2] = ((0b111 & (regs[2] >> 5)) << 2) | (0b11 & (regs[3] >> 0));
    g_st[0] = 0b11111 & regs[0];
    g_st[1] = 0b11111 & regs[1];
    g_st[2] = 0b11111 & regs[2];

    a_st[0] &= (32 - 1);
    a_st[1] &= (32 - 1);
    a_st[2] &= (32 - 1);
    g_st[0] &= (32 - 1);
    g_st[1] &= (32 - 1);
    g_st[2] &= (32 - 1);

    ft_a[0] = 1000 * accelSelfTestEquation[a_st[0]];
    ft_a[1] = 1000 * accelSelfTestEquation[a_st[1]];
    ft_a[2] = 1000 * accelSelfTestEquation[a_st[2]];
    ft_g[0] = 1000 * gyroSelfTestEquation[g_st[0]];
    ft_g[1] = -1000 * gyroSelfTestEquation[g_st[1]];
    ft_g[2] = 1000 * gyroSelfTestEquation[g_st[2]];

    for (int i = 0; i < 3; ++i) {
        int str = accel_bias_st[i] - accel_bias_regular[i];
        float Change_from_factory_trim = (float) (str - ft_a[i]) / ft_a[i];
        if (Change_from_factory_trim > 0.14 || Change_from_factory_trim < -0.14) {
            INV_DEBUG("6050 accel[%d] self test fail,result = %f,it demands >-0.14 && <0.14", i,
                      Change_from_factory_trim);
            accel_result = 1;
        } else {
            INV_TRACE("6050 accel[%d] self test result = %f,it demands >-0.14 && <0.14", i,
                      Change_from_factory_trim);
        }
    }

    for (int i = 0; i < 3; ++i) {
        int str = gyro_bias_st[i] - gyro_bias_regular[i];
        float Change_from_factory_trim = (float) (str - ft_g[i]) / ft_g[i];
        if (Change_from_factory_trim > 0.14 || Change_from_factory_trim < -0.14) {
            INV_DEBUG("6050 gryo[%d] self test fail,result = %f,it demands >-0.14 && <0.14", i,
                      Change_from_factory_trim);
            gyro_result = 1;
        } else {
            INV_TRACE("6050 gryo[%d] self test result = %f,it demands >-0.14 && <0.14", i,
                      Change_from_factory_trim);
        }
    }

    //恢复原来的配置
    res |= IMU_Init((inv_imu_handle)_this, backup_cfg);
    return (gyro_result << 1) | accel_result | res;
}
bool MPU6050_DataReady(inv_mpu6050_handle _this) {
    uint8_t val = 0;
    IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_INT_STATUS, &val);
    return (val & 0x01) == 0x01;
}
int MPU6050_EnableDataReadyInt(inv_mpu6050_handle _this) {
    return IMU_ModifyReg((inv_imu_handle)_this, (uint8_t) MPU6050_INT_ENABLE, 0x01, 0x01);
}
int MPU6050_SoftReset(inv_mpu6050_handle _this) {
    if (!IMU_Detect((inv_imu_handle)_this)) { return -1; }
    int res = 0;
    uint8_t val;
    //复位
    res |= IMU_WriteReg((inv_imu_handle)_this, (uint8_t) MPU6050_PWR_MGMT_1, 0x80);
    //等待复位成功
    do {
        IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_PWR_MGMT_1, &val);
        INV_TRACE("0x%x at PWR_MGMT_1,wait it get 0x40", val);
    } while (val != 0x40);

    //唤起睡眠
    IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_PWR_MGMT_1, &val);
    IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_PWR_MGMT_1, &val);
    IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_PWR_MGMT_1, &val);
    IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_PWR_MGMT_1, &val);
    IMU_ReadReg((inv_imu_handle)_this, (uint8_t) MPU6050_PWR_MGMT_1, &val);
    res |= IMU_WriteRegVerified((inv_imu_handle)_this, (uint8_t) MPU6050_PWR_MGMT_1, 0x0);

    return res;
}
int MPU6050_ReadSensorBlocking(inv_mpu6050_handle _this) {
    _this->parents.i2cTransfer.direction = inv_i2c_direction_Read;
    _this->parents.i2cTransfer.subAddress = (uint8_t) MPU6050_ACCEL_XOUT_H;
    _this->parents.i2cTransfer.data = _this->buf;
    _this->parents.i2cTransfer.dataSize = 14;
    return _this->parents.i2c.masterTransferBlocking(&_this->parents.i2cTransfer);
}
int MPU6050_ReadSensorNonBlocking(inv_mpu6050_handle _this) {
    _this->parents.i2cTransfer.direction = inv_i2c_direction_Read;
    _this->parents.i2cTransfer.subAddress = (uint8_t) MPU6050_ACCEL_XOUT_H;
    _this->parents.i2cTransfer.data = _this->buf;
    _this->parents.i2cTransfer.dataSize = 14;
    return _this->parents.i2c.masterTransferNonBlocking(&_this->parents.i2cTransfer);
}
int MPU6050_Convert(inv_mpu6050_handle _this, float *array) {
    uint8_t *buf = _this->buf;
    array[0] = _this->accelUnit * ((int16_t) ((buf[0] << 8) | buf[1]));
    array[1] = _this->accelUnit * ((int16_t) ((buf[2] << 8) | buf[3]));
    array[2] = _this->accelUnit * ((int16_t) ((buf[4] << 8) | buf[5]));
    array[3] = _this->gyroUnit * ((int16_t) ((buf[8] << 8) | buf[9]));
    array[4] = _this->gyroUnit * ((int16_t) ((buf[10] << 8) | buf[11]));
    array[5] = _this->gyroUnit * ((int16_t) ((buf[12] << 8) | buf[13]));
    return 0;
}
int MPU6050_Convert2(inv_mpu6050_handle _this, int16_t *raw) {
    uint8_t *buf = _this->buf;
    raw[0] = ((int16_t) ((buf[0] << 8) | buf[1]));
    raw[1] = ((int16_t) ((buf[2] << 8) | buf[3]));
    raw[2] = ((int16_t) ((buf[4] << 8) | buf[5]));
    raw[3] = ((int16_t) ((buf[8] << 8) | buf[9]));
    raw[4] = ((int16_t) ((buf[10] << 8) | buf[11]));
    raw[5] = ((int16_t) ((buf[12] << 8) | buf[13]));
    return 0;
}
int MPU6050_Convert3(inv_mpu6050_handle _this, float *temp) {
    if (temp) {
        *temp = (float) ((int16_t) ((((int16_t)_this->buf[6] << 8) | _this->buf[7]) - 521)) / 340.0f + 35;
    }
    return 0;
}

//#if defined(__cplusplus) || defined(c_plusplus)
//}
//#endif

#endif //INV_XXX_ENABLE


