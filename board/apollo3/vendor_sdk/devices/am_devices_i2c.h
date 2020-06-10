#ifndef I2CMODE_H
#define I2CMODE_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Global definitions for the commands
//
//*****************************************************************************
#define WHO_AM_I_REGISTER 0x0F      //register
//#define SAD_ID            0x6A      //slave address

//*****************************************************************************
//
// Global type definitions.
//
//*****************************************************************************
typedef enum
{
    AM_DEVICES_I2C_STATUS_SUCCESS,
    AM_DEVICES_I2C_STATUS_ERROR
} am_devices_i2c_status_t;

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
uint32_t am_devices_i2c_init(uint32_t ui32Module, am_hal_iom_config_t *psIOMSettings, void **ppIomHandle);
uint32_t am_devices_i2c_term(uint32_t ui32Module);

uint32_t am_devices_i2c_read_id(uint32_t *pDeviceID);


uint32_t am_devices_i2c_blocking_write(uint16_t ui16SadAddress,
                                                       uint16_t ui16WriteAddress,
                                                       uint8_t *pui8TxBuffer,
                                                       uint16_t ui16NumBytes);


uint32_t am_devices_i2c_nonblocking_write(uint16_t ui16SadAddress,
                                                        uint16_t ui16WriteAddress,
                                                        uint8_t *pui8TxBuffer,
                                                        uint16_t ui16NumBytes,
                                                        am_hal_iom_callback_t pfnCallback,
                                                        void *pCallbackCtxt);


uint32_t am_devices_i2c_blocking_read(uint16_t ui16SadAddress,
                                                    uint16_t ui16ReadAddress,
                                                    uint8_t *pui8RxBuffer,
                                                    uint16_t ui16NumBytes);


uint32_t am_devices_i2c_nonblocking_read(uint16_t ui16SadAddress,
                                                       uint16_t ui16ReadAddress,
                                                       uint8_t *pui8RxBuffer,
                                                       uint16_t ui16NumBytes,
                                                       am_hal_iom_callback_t pfnCallback,
                                                       void * pCallbackCtxt);

#ifdef __cplusplus
}
#endif

#endif // I2CMODE_H

