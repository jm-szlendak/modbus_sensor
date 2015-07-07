#ifndef IMU_H_INCLUDED
#define IMU_H_INCLUDED

#include "stm32f3_discovery_l3gd20.h"
#include "stm32f3_discovery_lsm303dlhc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_i2c.h"


#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
#define PI                         (float)     3.14159265f

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */


/**
  * \brief  Initialize IMU module
  * \param  None
  * \retval None
  */
void imuHardwareSetup();

/**
  * \brief  Read acceleration
  * \param  buffer: pointer to float[3] array of measure result
  * \retval None
  */
void imuReadAcceleration(float* buffer);

/**
  * \brief  Read angular rate
  * \param  buffer: pointer to float[3] array of measure result
  * \retval None
  */
void imuReadAngularRate(float* buffer);

/**
  * \brief  Read magnetic field value
  * \param  buffer: pointer to float[3] array of measure result
  * \retval None
  */
void imuReadMagneticField(float* buffer);






#endif /* IMU_H_INCLUDED */
