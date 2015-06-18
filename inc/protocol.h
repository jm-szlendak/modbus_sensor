#ifndef PROTOCOL_H_INCLUDED
#define PROTOCOL_H_INCLUDED

#define protocolMAX_FRAME_SIZE (20)
#define TAG_MESSAGE         ('!')
#define TAG_MESSAGE_END     ('/')
#define TAG_DATA            (0x02)
#define TAG_DATA_END        (0x03)


#define DATA_ACC_X      1
#define DATA_ACC_Y      2
#define DATA_ACC_Z      3

#define DEVICE_MODBUS_ADDRESS       (0x01)

#define MODBUS_FUNCTION_READ_FLOAT  (66)
#define MODBUS_FUNCTION_SLEEP       (67)
#define MODBUS_FUNCTION_WAKEUP      (68)

typedef struct
{
    unsigned char frame[protocolMAX_FRAME_SIZE];
    uint8_t  size;
}rx_frame_t;

typedef struct
{
    float fAccelerationX;
    float fAccelerationY;
    float fAccelerationZ;

    float fMagneticFieldX;
    float fMagneticFieldY;
    float fMagneticFieldZ;

    float fGyroAngRateX;
    float fGyroAngRateY;
    float fGyroAngRateZ;

    float fMainVoltage_mV;

    unsigned long lSamplePeriod;
} t_BoardStatus;

extern t_BoardStatus g_BoardStatus;


typedef struct
{
    unsigned char ucID;
    unsigned char ucSize;
    unsigned char *pucValue;
} t_DataItem;

extern const t_DataItem g_RealTimeData[];
extern const unsigned long g_ulNumRealTimeData;

#endif /* PROTOCOL_H_INCLUDED */
