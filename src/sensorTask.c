#include "sensorTask.h"


#define sensorSTACK_SIZE (configMINIMAL_STACK_SIZE + 2UL )
#define sensorQUEUE_SIZE 20UL
#define sensorNO_WAIT 0UL
#define sensorSAMPLE_PERIOD 100UL



void vStartSensorTask( UBaseType_t uxPriority)
{
    xTaskCreate(vSensorTask, "Sensor", sensorSTACK_SIZE, NULL, uxPriority, &xSensorTaskHandle );
}
void vSensorTask(void* pvParameters)
{
    TickType_t xLastSampleTime;
	xLastSampleTime = xTaskGetTickCount();
    for(;;)
    {

        vTaskDelayUntil(&xLastSampleTime, sensorSAMPLE_PERIOD/portTICK_PERIOD_MS);

    }
}


/**
* @brief Read LSM303DLHC output register, and calculate the acceleration ACC=(1/SENSITIVITY)* (out_h*256+out_l)/16 (12 bit rappresentation)
* @param pnData: pointer to float buffer where to store data
* @retval None
*/
void vReadAcc(float* pfData)
{
    int16_t pnRawData[3];
    uint8_t ctrlx[2];
    uint8_t buffer[6], cDivider;
    uint8_t i = 0;
    float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;

    /* Read the register content */
    LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);
    LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);

    if(ctrlx[1]&0x40)
        cDivider=64;
    else
        cDivider=16;

    /* check in the control register4 the data alignment*/
    if(!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) /* Little Endian Mode or FIFO mode */
    {
        for(i=0; i<3; i++)
        {
            pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i])/cDivider;
        }
    }
    else /* Big Endian Mode */
    {
        for(i=0; i<3; i++)
            pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])/cDivider;
    }
    /* Read the register content */
    LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);


    if(ctrlx[1]&0x40)
    {
        /* FIFO mode */
        LSM_Acc_Sensitivity = 0.25;
    }
    else
    {
        /* normal mode */
        /* switch the sensitivity value set in the CRTL4*/
        switch(ctrlx[0] & 0x30)
        {
        case LSM303DLHC_FULLSCALE_2G:
            LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
            break;
        case LSM303DLHC_FULLSCALE_4G:
            LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
            break;
        case LSM303DLHC_FULLSCALE_8G:
            LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
            break;
        case LSM303DLHC_FULLSCALE_16G:
            LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
            break;
        }
    }

    /* Obtain the mg value for the three axis */
    for(i=0; i<3; i++)
    {
        pfData[i]=(float)pnRawData[i]/LSM_Acc_Sensitivity;
    }

}


/**
  * @brief  Calculate the angular Data rate Gyroscope.
  * @param  pfData : Data out pointer
  * @retval None
  */
void vReadGyro (float* pfData)
{
    uint8_t tmpbuffer[6] = {0};
    int16_t RawData[3] = {0};
    uint8_t tmpreg = 0;
    float sensitivity = 0;
    int i =0;

    L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
    L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);

    /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
    if(!(tmpreg & 0x40))
    {
        for(i=0; i<3; i++)
        {
            RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
        }
    }
    else
    {
        for(i=0; i<3; i++)
        {
            RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
        }
    }

    /* Switch the sensitivity value set in the CRTL4 */
    switch(tmpreg & 0x30)
    {
    case 0x00:
        sensitivity=L3G_Sensitivity_250dps;
        break;

    case 0x10:
        sensitivity=L3G_Sensitivity_500dps;
        break;

    case 0x20:
        sensitivity=L3G_Sensitivity_2000dps;
        break;
    }
    /* divide by sensitivity */
    for(i=0; i<3; i++)
    {
        pfData[i]=(float)RawData[i]/sensitivity;
    }
}


/**
  * @brief  calculate the magnetic field Magn.
  * @param  pfData: pointer to the data out
  * @retval None
  */
void vReadMagneticField (float* pfData)
{
    static uint8_t buffer[6] = {0};
    uint8_t CTRLB = 0;
    uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
    uint8_t i =0;
    LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB, 1);

    LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 1);
    LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, buffer+1, 1);
    LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, buffer+2, 1);
    LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, buffer+3, 1);
    LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, buffer+4, 1);
    LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, buffer+5, 1);
    /* Switch the sensitivity set in the CRTLB*/
    switch(CTRLB & 0xE0)
    {
    case LSM303DLHC_FS_1_3_GA:
        Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
        Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
        break;
    case LSM303DLHC_FS_1_9_GA:
        Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
        Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
        break;
    case LSM303DLHC_FS_2_5_GA:
        Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
        Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
        break;
    case LSM303DLHC_FS_4_0_GA:
        Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
        Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
        break;
    case LSM303DLHC_FS_4_7_GA:
        Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
        Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
        break;
    case LSM303DLHC_FS_5_6_GA:
        Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
        Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
        break;
    case LSM303DLHC_FS_8_1_GA:
        Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
        Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
        break;
    }

    for(i=0; i<2; i++)
    {
        pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])*1000)/Magn_Sensitivity_XY;
    }
    pfData[2]=(float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/Magn_Sensitivity_Z;
}

