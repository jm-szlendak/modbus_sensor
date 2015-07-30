#ifndef PROTOCOL_H_INCLUDED
#define PROTOCOL_H_INCLUDED

#define protocolMAX_FRAME_SIZE (40)


#define DEVICE_MODBUS_ADDRESS       (0x01)

#define MODBUS_FUNCTION_READ_FLOAT      (66)
#define MODBUS_FUNCTION_SLEEP           (67)
#define MODBUS_FUNCTION_WAKEUP          (68)
#define MODBUS_FUNCTION_READ_REGISTER   (69)

#define MB_INDX_DEV_ADDRESS            (0)
#define MB_INDX_FUNCTION_CODE          (1)
#define MB_INDX_DATA                   (2)
#define MB_INDX_SIZE                   (3)

typedef enum
{
    BUTTON_STATE,

    ACCELERATION_X,
    ACCELERATION_Y,
    ACCELERATION_Z,

    GYRO_ANG_RATE_X,
    GYRO_ANG_RATE_Y,
    GYRO_ANG_RATE_Z,

    MAGNETIC_FIELD_X,
    MAGNETIC_FIELD_Y,
    MAGNETIC_FIELD_Z,

    ANGLE_THETA,
    ANGLE_PSI,
    ANGLE_RHO,

    MAIN_VOLTAGE,
    NUMBER_OF_FIELDS
} board_status_enum;

typedef struct
{
    unsigned char frame[protocolMAX_FRAME_SIZE];
    uint8_t  size;
}modbus_frame_t;





#endif /* PROTOCOL_H_INCLUDED */
