#include "imu.h"




static void lsm303dlhcHardwareInit()
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

static void l3g20HardwareInit()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Enable the SPI periph */
  RCC_APB2PeriphClockCmd(L3GD20_SPI_CLK, ENABLE);

  /* Enable SCK, MOSI and MISO GPIO clocks */
  RCC_AHBPeriphClockCmd(L3GD20_SPI_SCK_GPIO_CLK | L3GD20_SPI_MISO_GPIO_CLK | L3GD20_SPI_MOSI_GPIO_CLK, ENABLE);

  /* Enable CS  GPIO clock */
  RCC_AHBPeriphClockCmd(L3GD20_SPI_CS_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(L3GD20_SPI_SCK_GPIO_PORT, L3GD20_SPI_SCK_SOURCE, L3GD20_SPI_SCK_AF);
  GPIO_PinAFConfig(L3GD20_SPI_MISO_GPIO_PORT, L3GD20_SPI_MISO_SOURCE, L3GD20_SPI_MISO_AF);
  GPIO_PinAFConfig(L3GD20_SPI_MOSI_GPIO_PORT, L3GD20_SPI_MOSI_SOURCE, L3GD20_SPI_MOSI_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_SCK_PIN;
  GPIO_Init(L3GD20_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  L3GD20_SPI_MOSI_PIN;
  GPIO_Init(L3GD20_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_MISO_PIN;
  GPIO_Init(L3GD20_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(L3GD20_SPI);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(L3GD20_SPI, &SPI_InitStructure);

  /* Configure the RX FIFO Threshold */
  SPI_RxFIFOThresholdConfig(L3GD20_SPI, SPI_RxFIFOThreshold_QF);
  /* Enable SPI1  */
  SPI_Cmd(L3GD20_SPI, ENABLE);

  /* Configure GPIO PIN for Lis Chip select */
  GPIO_InitStructure.GPIO_Pin = L3GD20_SPI_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(L3GD20_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  GPIO_SetBits(L3GD20_SPI_CS_GPIO_PORT, L3GD20_SPI_CS_PIN);

}

static uint8_t lsm303dlhcWriteRegister(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer)
{
      uint16_t LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
      while(LSM303DLHC_I2C->ISR & I2C_ISR_BUSY)
      {
            if((LSM303DLHC_Timeout--) == 0) return 0;
      }
        /* Configure slave address, nbytes, reload, end mode and start or stop generation */
      I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

      /* Wait until TXIS flag is set */
      LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
      while(!(LSM303DLHC_I2C->ISR & I2C_ISR_TXIS))
      {
        if((LSM303DLHC_Timeout--) == 0) return 0;
      }

      /* Send Register address */
      I2C_SendData(LSM303DLHC_I2C, (uint8_t) RegAddr);

      /* Wait until TCR flag is set */
      LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
      while(!(LSM303DLHC_I2C->ISR & I2C_ISR_TCR))
      {
        if((LSM303DLHC_Timeout--) == 0) return 0;
      }

      /* Configure slave address, nbytes, reload, end mode and start or stop generation */
      I2C_TransferHandling(LSM303DLHC_I2C, DeviceAddr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

      /* Wait until TXIS flag is set */
      LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
      while(!(LSM303DLHC_I2C->ISR & I2C_ISR_TXIS))
      {
        if((LSM303DLHC_Timeout--) == 0) return 0;
      }

      /* Write data to TXDR */
      I2C_SendData(LSM303DLHC_I2C, *pBuffer);

      /* Wait until STOPF flag is set */
      LSM303DLHC_Timeout = LSM303DLHC_LONG_TIMEOUT;
      while(!(LSM303DLHC_I2C->ISR & I2C_ISR_STOPF))
      {
        if((LSM303DLHC_Timeout--) == 0) return 0;
      }

      /* Clear STOPF flag */
      LSM303DLHC_I2C->ICR |= I2C_ICR_STOPCF;

      return LSM303DLHC_OK;
}

static uint16_t lsm303dlhcReadRegister(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
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



/**
  * \brief  Sends a Byte through the SPI interface and return the Byte received
  *         from the SPI bus.
  * \param  Byte : Byte send.
  * \retval The received byte value
  */
static uint8_t l3gd20SendByte(uint8_t byte)
{
  /// Loop while DR register in not empty
  uint32_t L3GD20Timeout = L3GD20_FLAG_TIMEOUT;
  while (SPI_I2S_GetFlagStatus(L3GD20_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {
    if((L3GD20Timeout--) == 0) return L3GD20_TIMEOUT_UserCallback();
  }

  /// Send a Byte through the SPI peripheral
  SPI_SendData8(L3GD20_SPI, byte);

  /// Wait to receive a Byte
  L3GD20Timeout = L3GD20_FLAG_TIMEOUT;
  while (SPI_I2S_GetFlagStatus(L3GD20_SPI, SPI_I2S_FLAG_RXNE) == RESET)
  {
    if((L3GD20Timeout--) == 0) return L3GD20_TIMEOUT_UserCallback();
  }

  /// Return the Byte read from the SPI bus
  return (uint8_t)SPI_ReceiveData8(L3GD20_SPI);
}


/**
  * \brief  Writes one byte to the L3GD20.
  * \param  pBuffer : pointer to the buffer  containing the data to be written to the L3GD20.
  * \param  WriteAddr : L3GD20's internal address to write to.
  * \param  NumByteToWrite: Number of bytes to write.
  * \retval None
  */
static void l3gd20WriteRegister(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
     /** Configure the MS bit:
       * - When 0, the address will remain unchanged in multiple read/write commands.
       * - When 1, the address will be auto incremented in multiple read/write commands. */

    if(NumByteToWrite > 0x01)
    {
        WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
    }
    /// Set chip select Low at the start of the transmission.
    L3GD20_CS_LOW();

    /// Send the Address of the indexed register
    l3gd20SendByte(WriteAddr);
    /// Send the data that will be written into the device
    while(NumByteToWrite >= 0x01)
    {
        l3gd20SendByte(*pBuffer);
        NumByteToWrite--;
        pBuffer++;
    }

    /// Set chip select High at the end of the transmission
    L3GD20_CS_HIGH();
}

/**
  * \brief  Reads a block of data from the L3GD20.
  * \param  pBuffer : pointer to the buffer that receives the data read from the L3GD20.
  * \param  ReadAddr : L3GD20's internal address to read from.
  * \param  NumByteToRead : number of bytes to read from the L3GD20.
  * \retval None
  */
void l3gd20ReadRegister(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
     /** Configure the MS bit:
       * - When 0, the address will remain unchanged in multiple read/write commands.
       * - When 1, the address will be auto incremented in multiple read/write commands.
       */
    if(NumByteToRead > 0x01)
    {
        ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
    }
    else
    {
        ReadAddr |= (uint8_t)READWRITE_CMD;
    }
    /// Set chip select Low at the start of the transmission
    L3GD20_CS_LOW();

    /// Send the Address of the indexed register
    l3gd20SendByte(ReadAddr);

    /// Receive the data that will be read from the device (MSB First)
    while(NumByteToRead > 0x00)
    {
        /// Send dummy byte (0x00) to generate the SPI clock to L3GD20 (Slave device)
        *pBuffer = l3gd20SendByte(DUMMY_BYTE);
        NumByteToRead--;
        pBuffer++;
    }

    /// Set chip select High at the end of the transmission
    L3GD20_CS_HIGH();
}

static void accelerometerInit()
{
    uint8_t ctrl1 = 0x00, ctrl2 = 0x00, ctrl4 = 0x00;

    lsm303dlhcHardwareInit();
    /// Set accelerometer register values:
    ctrl1 |=    LSM303DLHC_NORMAL_MODE |    /*!< power mode */
                LSM303DLHC_ODR_50_HZ |      /*!< data rate */
                LSM303DLHC_X_ENABLE |       /*!< X axis enable */
                LSM303DLHC_Y_ENABLE |       /*!< Y axis enable */
                LSM303DLHC_Z_ENABLE;        /*!< Z axis enable */

    ctrl2 |=    LSM303DLHC_HPM_NORMAL_MODE| /*!<High Pass Filter enable */
                LSM303DLHC_HPFCF_16;         /*!<Cutoff frequency */

    ctrl4 |=    LSM303DLHC_HR_ENABLE;       /*!<high resolution enable */

    /*!< Write register values to chip via I2C interface */
    lsm303dlhcWriteRegister(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A, &ctrl1);
    lsm303dlhcWriteRegister(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &ctrl2);
    lsm303dlhcWriteRegister(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, &ctrl4);
}


static void magnetometerInit()
{
    uint8_t cra_reg_m = 0, crb_reg_m = 0, mr_reg_m = 0;

    lsm303dlhcHardwareInit();
    /// Set accelerometer register values:
    cra_reg_m |= LSM303DLHC_TEMPSENSOR_ENABLE;      /**< Enable temperature sensor */
    cra_reg_m |= LSM303DLHC_ODR_75_HZ;              /**<Data rate 75 Hz */
    crb_reg_m |= LSM303DLHC_FS_1_3_GA;              /**<Sensing range = +-  1.3 [Gauss] */
    mr_reg_m  |= LSM303DLHC_CONTINUOS_CONVERSION;    /**<Continuous conversion mode */
    /// Write register values to chip via I2C interface
    lsm303dlhcWriteRegister(MAG_I2C_ADDRESS, LSM303DLHC_CRA_REG_M, &cra_reg_m);
    lsm303dlhcWriteRegister(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &crb_reg_m);
    lsm303dlhcWriteRegister(MAG_I2C_ADDRESS, LSM303DLHC_MR_REG_M, &mr_reg_m);
}

static void gyroInit()
{
    uint8_t ctrl1 = 0, ctrl2 = 0, ctrl4 = 0;

    /// Configure gyro hardware
    l3g20HardwareInit();

    ctrl1 |=    L3GD20_MODE_ACTIVE |                /**< Power on */
                L3GD20_AXES_ENABLE |                /**< Enable all axes */
                L3GD20_OUTPUT_DATARATE_1 |
                L3GD20_BANDWIDTH_4;                 /**< This settings give 95 Hz sampling frequency */

    ctrl2 |=    L3GD20_HPM_NORMAL_MODE_RES |        /**< High pass filter enable */
                L3GD20_HPFCF_0;                     /**< 7.2 Hz cutoff */

    ctrl4 |=    L3GD20_BlockDataUpdate_Continous|   /**< Data in registers updated continuously */
                L3GD20_BLE_LSB |                    /**< LSB First */
                L3GD20_FULLSCALE_500;               /**< 500dps range */

    /// Write settings to chip
    l3gd20WriteRegister(&ctrl1, L3GD20_CTRL_REG1_ADDR, 1);
    l3gd20WriteRegister(&ctrl2, L3GD20_CTRL_REG2_ADDR, 1);
    l3gd20WriteRegister(&ctrl4, L3GD20_CTRL_REG4_ADDR, 1);
}

void imuReadAcceleration(float* result)
{
    int16_t rawData[3];
    uint8_t buffer[6] = {0};
    /// Read data registers of accelerometer
    lsm303dlhcReadRegister(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);

    /// Data is stored in 2 8bit registers, we need to join them into one number
    uint8_t i = 0;
    for(i = 0; i<3; i++)
    {
        rawData[i] = ((int16_t)((uint16_t)buffer[2*i+1]<<8) + buffer[2*i])/16;
        result[i] = (float)rawData[i]/LSM_Acc_Sensitivity_2g;
    }

}

void imuReadMagneticField(float* result)
{
    uint8_t buffer[6] = {0};
    /// Read data registers of magnetometer
    lsm303dlhcReadRegister(MAG_I2C_ADDRESS,LSM303DLHC_OUT_X_H_M, buffer, 6);

    /// Data is stored in 2 8bit registers, we need to join them into one number
    uint8_t i;
    for(i=0; i<2; i++)
    {
        result[i]=(float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])*1000)/LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
    }
    result[2]=(float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
}

void imuReadAngularRate(float* result)
{
    uint8_t buffer[6] = {0};
    int16_t rawData = 0;

    /// Read data registers of gyro
    l3gd20ReadRegister(buffer,L3GD20_OUT_X_L_ADDR,6);

    /// Data is stored in 2 8bit registers, we need to join them into one number
    uint8_t i;
    for(i=0; i<3; i++)
    {
        rawData = (int16_t)(((uint16_t)buffer[2*i+1] << 8) + buffer[2*i]);
        result[i] = (float) rawData / L3G_Sensitivity_500dps;
    }

}

void imuHardwareSetup()
{
    magnetometerInit();
    accelerometerInit();
    gyroInit();
}

