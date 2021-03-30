//
// Created by 17616 on 2021/3/30.
//

#ifndef INV_IMU_INV_ICM20600_H
#define INV_IMU_INV_ICM20600_H
#include "inv_icm20602.h"

typedef struct __inv_icm20600 {
    inv_icm20602 parents;
} inv_icm20600, *inv_icm20600_handle;



void ICM20600_Destruct(inv_icm20600_handle this) { ICM20602_Destruct((void *) this); }
inv_icm20600_handle ICM20600_Construct(inv_i2c _i2c, uint16_t _addr);
inv_icm20600_handle ICM20600_Construct2(inv_spi _spi);



bool ICM20600_Detect(inv_icm20602_handle this);
const char *ICM20600_Report(inv_icm20602_handle this) { return " icm20600"; }


#endif //INV_IMU_INV_ICM20600_H
