#ifndef INV_IMU_INV_ICM20600_H
#define INV_IMU_INV_ICM20600_H
#include "drv_imu_inv_icm20602.h"
#if INV_ICM20600_ENABLE

//#if defined(__cplusplus) || defined(c_plusplus)
//extern "C"{
//#endif

typedef struct __inv_icm20600 {
    inv_icm20602 parents;
} inv_icm20600, *inv_icm20600_handle;


inline void ICM20600_Destruct(inv_icm20600_handle _this) { ICM20602_Destruct((void *) _this); }
inv_icm20600_handle ICM20600_ConstructI2C(inv_i2c _i2c, uint8_t _addr);
inv_icm20600_handle ICM20600_ConstructSPI(inv_spi _spi);



bool ICM20600_Detect(inv_icm20602_handle _this);
inline const char *ICM20600_Report(inv_icm20602_handle _this) { return " icm20600"; }

//#if defined(__cplusplus) || defined(c_plusplus)
//}
//#endif

#endif //INV_XXX_ENABLE


#endif //INV_IMU_INV_ICM20600_H

