#ifndef INV_IMU_INV_ICM20600_H
#define INV_IMU_INV_ICM20600_H
#include "drv_imu_inv_icm20602.h"
#if defined(INV_ICM20600_ENABLE)&&(INV_ICM20600_ENABLE>0U)

//#if defined(__cplusplus) || defined(c_plusplus)
//extern "C"{
//#endif

typedef struct __inv_icm20600 {
    inv_icm20602_t parents;
} inv_icm20600_t, *inv_icm20600_handle_t;

#ifdef __cplusplus
extern "C"{
#endif

void ICM20600_Destruct(inv_icm20600_handle_t _this);
inv_icm20600_handle_t ICM20600_ConstructI2C(inv_i2c_t _i2c, uint8_t _addr);
inv_icm20600_handle_t ICM20600_ConstructSPI(inv_spi_t _spi);

bool ICM20600_Detect(inv_icm20602_handle_t _this);
const char *ICM20600_Report(inv_icm20602_handle_t _this);

#if defined(__cplusplus)
}
#endif

#endif //INV_XXX_ENABLE


#endif //INV_IMU_INV_ICM20600_H

