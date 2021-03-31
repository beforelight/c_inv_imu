#if defined(__cplusplus) || defined(c_plusplus)
extern "C"{
#endif

#include "inv_icm20602.h"
#if INV_ICM20602_ENABLE
const int DEF_ST_PRECISION = 1000;
const int DEF_GYRO_CT_SHIFT_DELTA = 500;
const int DEF_ACCEL_ST_SHIFT_DELTA = 500;
/* Gyro Offset Max Value (dps) */
const int DEF_GYRO_OFFSET_MAX = 20;
/* Gyro Self Test Absolute Limits ST_AL (dps) */
const int DEF_GYRO_ST_AL = 60;
/* Accel Self Test Absolute Limits ST_AL (mg) */
const int DEF_ACCEL_ST_AL_MIN = 225;
const int DEF_ACCEL_ST_AL_MAX = 675;

const uint16_t sSelfTestEquation[256] = {
        2620, 2646, 2672, 2699, 2726, 2753, 2781, 2808,
        2837, 2865, 2894, 2923, 2952, 2981, 3011, 3041,
        3072, 3102, 3133, 3165, 3196, 3228, 3261, 3293,
        3326, 3359, 3393, 3427, 3461, 3496, 3531, 3566,
        3602, 3638, 3674, 3711, 3748, 3786, 3823, 3862,
        3900, 3939, 3979, 4019, 4059, 4099, 4140, 4182,
        4224, 4266, 4308, 4352, 4395, 4439, 4483, 4528,
        4574, 4619, 4665, 4712, 4759, 4807, 4855, 4903,
        4953, 5002, 5052, 5103, 5154, 5205, 5257, 5310,
        5363, 5417, 5471, 5525, 5581, 5636, 5693, 5750,
        5807, 5865, 5924, 5983, 6043, 6104, 6165, 6226,
        6289, 6351, 6415, 6479, 6544, 6609, 6675, 6742,
        6810, 6878, 6946, 7016, 7086, 7157, 7229, 7301,
        7374, 7448, 7522, 7597, 7673, 7750, 7828, 7906,
        7985, 8065, 8145, 8227, 8309, 8392, 8476, 8561,
        8647, 8733, 8820, 8909, 8998, 9088, 9178, 9270,
        9363, 9457, 9551, 9647, 9743, 9841, 9939, 10038,
        10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,
        10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,
        11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,
        12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,
        13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,
        15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,
        16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,
        17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,
        19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
        20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,
        22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,
        24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,
        26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,
        28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,
        30903, 31212, 31524, 31839, 32157, 32479, 32804
};

inv_imu_vector_table icm20602_VectorTable =
        {
                .Init = (void *) ICM20602_Init,
                .Detect =(void *) ICM20602_Detect,
                .SelfTest =(void *) ICM20602_SelfTest,
                .Report =(void *) ICM20602_Report,
                .DataReady =(void *) ICM20602_DataReady,
                .EnableDataReadyInt =(void *) ICM20602_EnableDataReadyInt,
                .SoftReset =(void *) ICM20602_SoftReset,
                .ReadSensorBlocking =(void *) ICM20602_ReadSensorBlocking,
                .ReadSensorNonBlocking =(void *) ICM20602_ReadSensorNonBlocking,
                .Convert =(void *) ICM20602_Convert,
                .Convert2 =(void *) ICM20602_Convert2,
                .Convert3 =(void *) ICM20602_Convert3,
                .IsOpen =(void *) _IMU_IsOpen
        };


inv_icm20602_handle ICM20602_Construct(inv_i2c _i2c, uint16_t _addr) {
    inv_icm20602_handle rtv = (void *) INV_REALLOC(IMU_Construct(_i2c, _addr), sizeof(inv_icm20602));
    memset((void *) ((char *) rtv + sizeof(inv_icm20602) - sizeof(inv_imu)), 0, sizeof(inv_icm20602) - sizeof(inv_imu));
    rtv->parents.vtable = &icm20602_VectorTable;
    rtv->buf = rtv->rxbuf + 1;
    return rtv;
}
inv_icm20602_handle ICM20602_Construct2(inv_spi _spi) {
    inv_icm20602_handle rtv = (void *) INV_REALLOC(IMU_Construct2(_spi), sizeof(inv_icm20602));
    memset((void *) ((char *) rtv + sizeof(inv_icm20602) - sizeof(inv_imu)), 0, sizeof(inv_icm20602) - sizeof(inv_imu));
    rtv->parents.vtable = &icm20602_VectorTable;
    rtv->buf = rtv->rxbuf + 1;
    return rtv;
}
int ICM20602_Init(inv_icm20602_handle this, inv_imu_config _cfg) {
    this->parents.cfg = _cfg;
    this->parents.isOpen = false;
    int res = 0;
    int bw;
    int fs;
    float unit;
    float unit_from;
    if (!IMU_Detect((inv_imu_handle) this)) { return -1; }
    //软复位
    res |= IMU_SoftReset((inv_imu_handle) this);

    //打开所有传感器
    res |= IMU_WriteRegVerified((inv_imu_handle) this, (uint8_t) ICM20602_PWR_MGMT_2, 0);

    //1khz采样率
    res |= IMU_WriteRegVerified((inv_imu_handle) this, (uint8_t) ICM20602_SMPLRT_DIV, 0);

    //配置陀螺仪lpf
    _InvGetMapVal(ICM20602_GBW_MAP, this->parents.cfg.gyroBandwidth, bw);
    res |= IMU_WriteRegVerified((inv_imu_handle) this, (uint8_t) ICM20602_CONFIG, bw);

    //配置陀螺仪量程和单位
    _InvGetMapVal(mpu_gyro_unit_dps_map, this->parents.cfg.gyroFullScale, unit);
    _InvGetMapVal(mpu_gyro_unit_from_dps_map, this->parents.cfg.gyroUnit, unit_from);
    this->gyroUnit = unit * unit_from;

    _InvGetMapVal(mpu_gyro_fs_map, this->parents.cfg.gyroFullScale, fs);
    res |= IMU_WriteRegVerified((inv_imu_handle) this, (uint8_t) ICM20602_GYRO_CONFIG, fs << 3u);

    //配置加速度计量程和单位
    _InvGetMapVal(mpu_accel_unit_G_map, this->parents.cfg.accelFullScale, unit);
    _InvGetMapVal(mpu_accel_unit_from_G_map, this->parents.cfg.accelUnit, unit_from);
    this->accelUnit = unit * unit_from;

    _InvGetMapVal(mpu_accel_fs_map, this->parents.cfg.accelFullScale, fs);
    res |= IMU_WriteRegVerified((inv_imu_handle) this, (uint8_t) ICM20602_ACCEL_CONFIG, fs << 3u);

    //配置加速度计lpf
    _InvGetMapVal(ICM20602_ABW_MAP, this->parents.cfg.accelBandwidth, bw);
    res |= IMU_WriteRegVerified((inv_imu_handle) this, (uint8_t) ICM20602_ACCEL_CONFIG2, bw);

    //开启数据更新中断
    res |= IMU_EnableDataReadyInt((inv_imu_handle) this);

    if (res == 0) {
        this->parents.isOpen = true;
    }
    return res;
}
bool ICM20602_Detect(inv_icm20602_handle this) {
    uint8_t val = 0;
    if (this->parents.addrAutoDetect) { this->parents.i2cTransfer.slaveAddress = 0x68; }
    IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_WHO_AM_I, &val);
    if (0x12 == val) {
        return true;
    }
    val = 0;
    if (this->parents.addrAutoDetect) { this->parents.i2cTransfer.slaveAddress = 0x69; }
    IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_WHO_AM_I, &val);
    if (0x12 == val) {
        return true;
    }
    return false;
}
int ICM20602_SelfTest(inv_icm20602_handle this) {
    if (!IMU_IsOpen((inv_imu_handle) this)) { return -1; }
    int res = 0;
    inv_imu_config backup_cfg = this->parents.cfg;
    inv_imu_config st_cfg = IMU_ConfigDefault();
    st_cfg.gyroFullScale = MPU_FS_250dps;
    st_cfg.accelFullScale = MPU_FS_2G;
    st_cfg.accelBandwidth = MPU_ABW_99;
    st_cfg.gyroBandwidth = MPU_GBW_92;
    if (0 != IMU_Init((inv_imu_handle) this, st_cfg)) {
        IMU_Init((inv_imu_handle) this, backup_cfg);
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
    while (times--) { while (!IMU_DataReady((inv_imu_handle) this)) {}}//丢弃前20个数据
    times = 20;
    while (times--) {
        while (!IMU_DataReady((inv_imu_handle) this)) {}
        res |= IMU_ReadSensorBlocking((inv_imu_handle) this);
        IMU_Convert2((inv_imu_handle) this, abuf);
        for (int i = 0; i < 3; ++i) {
            gyro_bias_regular[i] += gbuf[i];
            accel_bias_regular[i] += abuf[i];
        }
    }

    res |= IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_GYRO_CONFIG, &val);
    res |= IMU_WriteRegVerified((inv_imu_handle) this, (uint8_t) ICM20602_GYRO_CONFIG, val | (0b111 << 5));//打开陀螺仪自检
    res |= IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_ACCEL_CONFIG, &val);
    res |= IMU_WriteRegVerified((inv_imu_handle) this, (uint8_t) ICM20602_ACCEL_CONFIG, val | (0b111 << 5));//打开加速度计自检
    times = 20;
    while (times--) { while (!IMU_DataReady((inv_imu_handle) this)) {}}//丢弃前20个数据
    times = 20;
    while (times--) {
        while (!IMU_DataReady((inv_imu_handle) this)) {}
        res |= IMU_ReadSensorBlocking((inv_imu_handle) this);
        IMU_Convert2((inv_imu_handle) this, abuf);
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


    //计算加速度计自检结果
    uint8_t regs[3];
    int otp_value_zero = 0;
    int st_shift_prod[3], st_shift_cust[3], st_shift_ratio[3], i;
//    int result;

    res |= IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_SELF_TEST_X_ACCEL, regs);
    res |= IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_SELF_TEST_Y_ACCEL, regs + 1);
    res |= IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_SELF_TEST_Z_ACCEL, regs + 2);
    for (i = 0; i < 3; i++) {
        if (regs[i] != 0) {
            st_shift_prod[i] = sSelfTestEquation[regs[i] - 1];
        } else {
            st_shift_prod[i] = 0;
            otp_value_zero = 1;
        }
    }

    if (!otp_value_zero) {
        /* Self Test Pass/Fail Criteria A */
        for (i = 0; i < 3; i++) {
            st_shift_cust[i] = accel_bias_st[i] - accel_bias_regular[i];
            st_shift_ratio[i] = abs(st_shift_cust[i] / st_shift_prod[i] - DEF_ST_PRECISION);
            if (st_shift_ratio[i] > DEF_ACCEL_ST_SHIFT_DELTA) {
                //加速度计自检未通过
                accel_result = 1;
                INV_DEBUG("accel[%d] st fail,result = %d,it demands less than %d", i, st_shift_ratio[i],
                          DEF_ACCEL_ST_SHIFT_DELTA);
            } else {
                INV_TRACE("accel[%d] st result = %d,it demands less than %d", i, st_shift_ratio[i],
                          DEF_ACCEL_ST_SHIFT_DELTA);
            }
        }
    } else {
        /* Self Test Pass/Fail Criteria B */
        for (i = 0; i < 3; i++) {
            st_shift_cust[i] = abs(accel_bias_st[i] - accel_bias_regular[i]);
            if (st_shift_cust[i] < DEF_ACCEL_ST_AL_MIN * (32768 / 2000) * 1000
                || st_shift_cust[i] > DEF_ACCEL_ST_AL_MAX * (32768 / 2000) * 1000) {
                //加速度计自检未通过
                accel_result = 1;
                INV_DEBUG("accel[%d] st fail,result = %d,it demands <%d && >%d", i, st_shift_cust[i],
                          DEF_ACCEL_ST_AL_MAX * (32768 / 2000) * 1000, DEF_ACCEL_ST_AL_MIN * (32768 / 2000) * 1000);
            } else {
                INV_TRACE("accel[%d] st result = %d,it demands <%d && >%d", i, st_shift_cust[i],
                          DEF_ACCEL_ST_AL_MAX * (32768 / 2000) * 1000, DEF_ACCEL_ST_AL_MIN * (32768 / 2000) * 1000);
            }
        }
    }

    //计算陀螺仪自检结果
    res |= IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_SELF_TEST_X_GYRO, regs);
    res |= IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_SELF_TEST_Y_GYRO, regs + 1);
    res |= IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_SELF_TEST_Z_GYRO, regs + 2);
    for (i = 0; i < 3; i++) {
        if (regs[i] != 0) {
            st_shift_prod[i] = sSelfTestEquation[regs[i] - 1];
        } else {
            st_shift_prod[i] = 0;
            otp_value_zero = 1;
        }
    }

    for (i = 0; i < 3; i++) {
        st_shift_cust[i] = gyro_bias_st[i] - gyro_bias_regular[i];
        if (!otp_value_zero) {
            /* Self Test Pass/Fail Criteria A */
            if (st_shift_cust[i] < DEF_GYRO_CT_SHIFT_DELTA * st_shift_prod[i]) {
                //陀螺仪自检没过
                gyro_result = 1;
                INV_DEBUG("gyro[%d] st fail,result = %d,it demands greater than %d", i, st_shift_cust[i],
                          DEF_GYRO_CT_SHIFT_DELTA * st_shift_prod[i]);
            } else {
                INV_TRACE("gyro[%d] st result = %d,it demands greater than %d", i, st_shift_cust[i],
                          DEF_GYRO_CT_SHIFT_DELTA * st_shift_prod[i]);
            }
        } else {
            /* Self Test Pass/Fail Criteria B */
            if (st_shift_cust[i] < DEF_GYRO_ST_AL * (32768 / 250) * DEF_ST_PRECISION) {
                //陀螺仪自检没过
                gyro_result = 1;
                INV_DEBUG("gyro[%d] st fail,result = %d,it demands greater than %d", i, st_shift_cust[i],
                          DEF_GYRO_ST_AL * (32768 / 250) * DEF_ST_PRECISION);
            } else {
                INV_TRACE("gyro[%d] st result = %d,it demands greater than %d", i, st_shift_cust[i],
                          DEF_GYRO_ST_AL * (32768 / 250) * DEF_ST_PRECISION);
            }
        }
    }

    if (gyro_result == 0) {
        /* Self Test Pass/Fail Criteria C */
        for (i = 0; i < 3; i++) {
            if (abs(gyro_bias_regular[i]) > DEF_GYRO_OFFSET_MAX * (32768 / 250) * DEF_ST_PRECISION)
                //陀螺仪自检没过
            {
                gyro_result = 1;
                INV_DEBUG("gyro[%d] st fail,result = %d,ift demands less than %d", i, (int) abs(gyro_bias_regular[i]),
                          DEF_GYRO_OFFSET_MAX * (32768 / 250) * DEF_ST_PRECISION);
            } else {
                INV_TRACE("gyro[%d] st result = %d,it demands less than %d", i, (int) abs(gyro_bias_regular[i]),
                          DEF_GYRO_OFFSET_MAX * (32768 / 250) * DEF_ST_PRECISION);
            }
        }
    }

    //恢复原来的配置
    res |= IMU_Init((inv_imu_handle) this, backup_cfg);
    return res | (gyro_result << 1) | accel_result;
}
bool ICM20602_DataReady(inv_icm20602_handle this) {
    uint8_t val = 0;
    IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_INT_STATUS, &val);
    return (val & 0x01) == 0x01;
}
int ICM20602_EnableDataReadyInt(inv_icm20602_handle this) {
    return IMU_ModifyReg((inv_imu_handle) this, (uint8_t) ICM20602_INT_ENABLE, 0x01, 0x01);
}
int ICM20602_SoftReset(inv_icm20602_handle this) {
    if (!IMU_Detect((inv_imu_handle) this)) { return -1; }
    int res = 0;
    uint8_t val;
    //复位
    res |= IMU_WriteReg((inv_imu_handle) this, (uint8_t) ICM20602_PWR_MGMT_1, 0x80);
    //等待复位成功
    do {
        IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_PWR_MGMT_1, &val);
        INV_TRACE("0x%x at PWR_MGMT_1,wait it get 0x41", val);
    } while (val != 0x41);

    //唤起睡眠
    IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_PWR_MGMT_1, &val);
    IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_PWR_MGMT_1, &val);
    IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_PWR_MGMT_1, &val);
    IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_PWR_MGMT_1, &val);
    IMU_ReadReg((inv_imu_handle) this, (uint8_t) ICM20602_PWR_MGMT_1, &val);
    res |= IMU_WriteRegVerified((inv_imu_handle) this, (uint8_t) ICM20602_PWR_MGMT_1, 0x1);

    return res;
}
int ICM20602_ReadSensorBlocking(inv_icm20602_handle this) {
    int res;
    if (this->parents.i2c.masterTransferBlocking != NULL) {
        this->parents.i2cTransfer.subAddress = (uint8_t) ICM20602_ACCEL_XOUT_H;
        this->parents.i2cTransfer.data = this->buf;
        this->parents.i2cTransfer.dataSize = 14;
        this->parents.i2cTransfer.direction = inv_i2c_direction_Read;
        res = this->parents.i2c.masterTransferBlocking(&this->parents.i2cTransfer);
        if (res != 0) {
            INV_DEBUG("i2c read return code = %d", res);
        }
    } else {
        this->txbuf[0] = (1U << 7U) | ((uint8_t) ICM20602_ACCEL_XOUT_H & 0x7fU);
        this->parents.spiTransfer.dataSize = 15;
        this->parents.spiTransfer.rxData = this->rxbuf;
        this->parents.spiTransfer.txData = this->txbuf;
        res = this->parents.spi.masterTransferBlocking(&this->parents.spiTransfer);
        if (res != 0) {
            INV_DEBUG("spi read return code = %d", res);
        }
    }
    return res;
}
int ICM20602_ReadSensorNonBlocking(inv_icm20602_handle this) {
    int res;
    if (this->parents.i2c.masterTransferBlocking != NULL) {
        this->parents.i2cTransfer.subAddress = (uint8_t) ICM20602_ACCEL_XOUT_H;
        this->parents.i2cTransfer.data = this->buf;
        this->parents.i2cTransfer.dataSize = 14;
        this->parents.i2cTransfer.direction = inv_i2c_direction_Read;
        res = this->parents.i2c.masterTransferNonBlocking(&this->parents.i2cTransfer);
        if (res != 0) {
            INV_DEBUG("i2c read return code = %d", res);
        }
    } else {
        this->txbuf[0] = (1U << 7U) | ((uint8_t) ICM20602_ACCEL_XOUT_H & 0x7fU);
        this->parents.spiTransfer.dataSize = 15;
        this->parents.spiTransfer.rxData = this->rxbuf;
        this->parents.spiTransfer.txData = this->txbuf;
        res = this->parents.spi.masterTransferNonBlocking(&this->parents.spiTransfer);
        if (res != 0) {
            INV_DEBUG("spi read return code = %d", res);
        }
    }
    return res;
}
int ICM20602_Convert(inv_icm20602_handle this, float *array) {
    uint8_t *buf = this->buf;
    array[0] = this->accelUnit * ((int16_t) ((buf[0] << 8) | buf[1]));
    array[1] = this->accelUnit * ((int16_t) ((buf[2] << 8) | buf[3]));
    array[2] = this->accelUnit * ((int16_t) ((buf[4] << 8) | buf[5]));
    array[3] = this->gyroUnit * ((int16_t) ((buf[8] << 8) | buf[9]));
    array[4] = this->gyroUnit * ((int16_t) ((buf[10] << 8) | buf[11]));
    array[5] = this->gyroUnit * ((int16_t) ((buf[12] << 8) | buf[13]));
    return 0;
}
int ICM20602_Convert2(inv_icm20602_handle this, int16_t *raw) {
    uint8_t *buf = this->buf;
    raw[0] = ((int16_t) ((buf[0] << 8) | buf[1]));
    raw[1] = ((int16_t) ((buf[2] << 8) | buf[3]));
    raw[2] = ((int16_t) ((buf[4] << 8) | buf[5]));
    raw[3] = ((int16_t) ((buf[8] << 8) | buf[9]));
    raw[4] = ((int16_t) ((buf[10] << 8) | buf[11]));
    raw[5] = ((int16_t) ((buf[12] << 8) | buf[13]));
    return 0;
}
int ICM20602_Convert3(inv_icm20602_handle this, float *temp) {
    if (temp) { *temp = (float) ((int16_t) (this->buf[6] << 8) | this->buf[7]) / 326.8f + 25.0f; }
    return 0;
}

#endif //INV_XXX_ENABLE

#if defined(__cplusplus) || defined(c_plusplus)
}
#endif