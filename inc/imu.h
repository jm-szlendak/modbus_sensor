#ifndef IMU_H_INCLUDED
#define IMU_H_INCLUDED

#include "stm32f3_discovery_l3gd20.h"
#include "stm32f3_discovery_lsm303dlhc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_i2c.h"

void imuHardwareSetup();

void imuReadAcceleration(float* buffer);
void imuReadAngularRate(float* buffer);
void imuReadMagneticField(float* buffer);



#endif /* IMU_H_INCLUDED */
