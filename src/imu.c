#include "imu.h"

void imuHardwareSetup()
{

}

void imuReadAcceleration(float* buffer);
void imuReadAngularRate(float* buffer);
void imuReadMagneticField(float* buffer);

static void accelerometerHardwareInit()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;

    RCC_APB1PeriphClockCmd(LSM303DLHC_I2C_CLK, ENABLE); //Enable I2C peripherial clock
    RCC_AHBPeriphClockCmd(LSM303DLHC_I2C_SCK_GPIO_CLK | LSM303DLHC_I2C_SDA_GPIO_CLK , ENABLE); //Enable SDA and SCL ports clock

    /* Configure SDA and SCL pins to do alternative function - I2C */
    GPIO_PinAFConfig(LSM303DLHC_I2C_SCK_GPIO_PORT, LSM303DLHC_I2C_SCK_SOURCE, LSM303DLHC_I2C_SCK_AF);
    GPIO_PinAFConfig(LSM303DLHC_I2C_SDA_GPIO_PORT, LSM303DLHC_I2C_SDA_SOURCE, LSM303DLHC_I2C_SDA_AF);

    /* Fill settings structure for pins */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //Alternative function mode
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //Push-pull output
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;     //Pull down resistor
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   //50Mhz speed

    /* I2C SCK pin configuration */
    GPIO_InitStructure.GPIO_Pin = LSM303DLHC_I2C_SCK_PIN;
    GPIO_Init(LSM303DLHC_I2C_SCK_GPIO_PORT, &GPIO_InitStructure);

    /* I2C SDA pin configuration */
    GPIO_InitStructure.GPIO_Pin =  LSM303DLHC_I2C_SDA_PIN;
    GPIO_Init(LSM303DLHC_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

    /* Fill settings structure for I2C peripherial */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;                      //I2C mode
    I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;   //Analog Filter enable
    I2C_InitStructure.I2C_DigitalFilter = 0x00;                     //Digital Filter disable
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;                       //Own address, used only in SLAVE mode
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;                     //Enable acknowledgement
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; //7bit addressing mode
    I2C_InitStructure.I2C_Timing = 0x00902025;                      //This magic number encapsulates all timing params

    /* Apply LSM303DLHC_I2C configuration after enabling it */
    I2C_Init(LSM303DLHC_I2C, &I2C_InitStructure);

    /* LSM303DLHC_I2C Peripheral Enable */
    I2C_Cmd(LSM303DLHC_I2C, ENABLE);

}

static uint8_t accelerometerWriteRegister(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer)
{
      uint16_t LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
      while(LSM303DLHC_I2C->ISR & I2C_ISR_BUSY)
      {
            if((LSM303DLHC_Timeout--) == 0) return accelerometer_TIMEOUT_UserCallback();
      }
        /* Configure slave address, nbytes, reload, end mode and start or stop generation */
      I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

      /* Wait until TXIS flag is set */
      LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
      while(!(LSM303DLHC_I2C->ISR & I2C_ISR_TXIS))
      {
        if((LSM303DLHC_Timeout--) == 0) return accelerometer_TIMEOUT_UserCallback();
      }

      /* Send Register address */
      I2C_SendData(LSM303DLHC_I2C, (uint8_t) RegAddr);

      /* Wait until TCR flag is set */
      LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
      while(!(LSM303DLHC_I2C->ISR & I2C_ISR_TCR))
      {
        if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
      }

      /* Configure slave address, nbytes, reload, end mode and start or stop generation */
      I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

      /* Wait until TXIS flag is set */
      LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
      while(!(LSM303DLHC_I2C->ISR & I2C_ISR_TXIS))
      {
        if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
      }

      /* Write data to TXDR */
      I2C_SendData(LSM303DLHC_I2C, *pBuffer);

      /* Wait until STOPF flag is set */
      LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
      while(!(LSM303DLHC_I2C->ISR & I2C_ISR_STOPF))
      {
        if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
      }

      /* Clear STOPF flag */
      LSM303DLHC_I2C->ICR |= I2C_ICR_STOPCF;

      return LSM303DLHC_OK;
}

static uint16_t accelerometerReadRegister(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{
  /* Test on BUSY Flag */
  uint16_t LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
  while(LSM303DLHC_I2C->ISR & I2C_ISR_BUSY)
  {
    if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
  while(!(LSM303DLHC_I2C->ISR & I2C_ISR_TXIS))
  {
    if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
  }

  if(NumByteToRead>1)
      RegAddr |= 0x80;


  /* Send Register address */
  I2C_SendData(LSM303DLHC_I2C, (uint8_t)RegAddr);

  /* Wait until TC flag is set */
  LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
  while(!(LSM303DLHC_I2C->ISR & I2C_ISR_TC))
  {
    if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, NumByteToRead, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

  /* Wait until all data are received */
  while (NumByteToRead)
  {
    /* Wait until RXNE flag is set */
    LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
    while(!(LSM303DLHC_I2C->ISR & I2C_ISR_RXNE))
    {
      if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
    }

    /* Read data from RXDR */
    *pBuffer = I2C_ReceiveData(LSM303DLHC_I2C);
    /* Point to the next location where the byte read will be saved */
    pBuffer++;

    /* Decrement the read bytes counter */
    NumByteToRead--;
  }

  /* Wait until STOPF flag is set */
  LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
  while(!(LSM303DLHC_I2C->ISR & I2C_ISR_STOPF))
  {
    if((LSM303DLHC_Timeout--) == 0) return LSM303DLHC_TIMEOUT_UserCallback();
  }

  /* Clear STOPF flag */
  LSM303DLHC_I2C->ICR |= I2C_ICR_STOPCF;

  /* If all operations OK */
  return LSM303DLHC_OK;
}

static void accelerometerInit()
{
    uint8_t ctrl1 = 0x00, ctrl2 = 0x00, ctrl4 = 0x00;

    accelerometerHardwareInit();
    //Set accelerometer register values:
    ctrl1 |=    LSM303DLHC_NORMAL_MODE |    // power mode
                LSM303DLHC_ODR_50_HZ |      // data rate
                LSM303DLHC_X_ENABLE |       // X axis enable
                LSM303DLHC_Y_ENABLE |       // Y axis enable
                LSM303DLHC_Z_ENABLE;        // Z axis enable

    ctrl2 |=    LSM303DLHC_HPM_NORMAL_MODE| //High Pass Filter enable
                LSM303DLHC_HPFCF_16;         //Cutoff frequency

    ctrl4 |=    LSM303DLHC_HR_ENABLE;       //high resolution enable

    // Write register values to chip via I2C interface
    accelerometerWriteRegister(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A, &ctrl1);
    accelerometerWriteRegister(ACC_I2C_ADDRESS, LLSM303DLHC_CTRL_REG2_A, &ctrl2);
    accelerometerWriteRegister(ACC_I2C_ADDRESS, LLSM303DLHC_CTRL_REG4_A, &ctrl4);
}
