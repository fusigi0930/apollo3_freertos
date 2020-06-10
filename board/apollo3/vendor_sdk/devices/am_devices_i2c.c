#include "am_mcu_apollo.h"
#include "am_devices_i2c.h"
#include "am_bsp.h"
#include "am_util_delay.h"

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
static void *g_pI2CIOMHandle;

//*****************************************************************************
//
// Generic Command Write function.
//
//*****************************************************************************
static uint32_t
am_device_command_write(uint8_t ui8DevAddr, uint32_t ui32InstrLen,
                        uint32_t ui32Instr, bool bCont,
                        uint32_t *pData, uint32_t ui32NumBytes)
{
    am_hal_iom_transfer_t Transaction;

    //
    // Create the transaction.
    //
    Transaction.ui32InstrLen    = ui32InstrLen;
    Transaction.ui32Instr       = ui32Instr;
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = ui32NumBytes;
    Transaction.pui32TxBuffer   = pData;
    Transaction.uPeerInfo.ui32I2CDevAddr = (ui8DevAddr >> 1);
    Transaction.bContinue       = bCont;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    //
    // Execute the transction over IOM.
    //
    if (am_hal_iom_blocking_transfer(g_pI2CIOMHandle, &Transaction))
    {
        return AM_DEVICES_I2C_STATUS_ERROR;
    }
    return AM_DEVICES_I2C_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Generic Command Read function.
//
//*****************************************************************************
static uint32_t
am_device_command_read(uint8_t ui8DevAddr, uint32_t ui32InstrLen, uint32_t ui32Instr,
                       bool bCont, uint32_t *pData, uint32_t ui32NumBytes)
{
    am_hal_iom_transfer_t  Transaction;

    //
    // Create the transaction.
    //
    Transaction.ui32InstrLen    = ui32InstrLen;
    Transaction.ui32Instr       = ui32Instr;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = ui32NumBytes;
    Transaction.pui32RxBuffer   = pData;
    Transaction.uPeerInfo.ui32I2CDevAddr = (ui8DevAddr >> 1);
    Transaction.bContinue       = bCont;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    //
    // Execute the transction over IOM.
    //
    if (am_hal_iom_blocking_transfer(g_pI2CIOMHandle, &Transaction))
    {
        return AM_DEVICES_I2C_STATUS_ERROR;
    }
    return AM_DEVICES_I2C_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Initialize the i2c driver.
//!
//! @param psIOMSettings  - IOM device structure describing the target spiflash.
//! @param pDMACtrlBuffer - DMA Transfer Control Buffer.
//!
//! This function should be called before any other i2c
//! functions. It is used to set tell the other functions how to communicate
//! with the external spiflash hardware.
//!
//! @return Status.
//
//*****************************************************************************
uint32_t
am_devices_i2c_init(uint32_t ui32Module, am_hal_iom_config_t *psIOMSettings, void **ppIomHandle)
{
    if ( ui32Module > AM_REG_IOM_NUM_MODULES )
    {
        return AM_DEVICES_I2C_STATUS_ERROR;
    }

    //
    // Enable fault detection.
    //
#if AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_FAULT_CAPTURE_ENABLE, 0);
#else // AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_fault_capture_enable();
#endif // AM_APOLLO3_MCUCTRL

    //
    // Initialize the IOM instance.
    // Enable power to the IOM instance.
    // Configure the IOM for Serial operation during initialization.
    // Enable the IOM.
    //
    if (am_hal_iom_initialize(ui32Module, &g_pI2CIOMHandle) ||
        am_hal_iom_power_ctrl(g_pI2CIOMHandle, AM_HAL_SYSCTRL_WAKE, false) ||
        am_hal_iom_configure(g_pI2CIOMHandle, psIOMSettings) ||
        am_hal_iom_enable(g_pI2CIOMHandle))
    {
        return AM_DEVICES_I2C_STATUS_ERROR;
    }
    else
    {
        //
        // Configure the IOM pins.
        //
        am_bsp_iom_pins_enable(ui32Module, AM_HAL_IOM_I2C_MODE);


        *ppIomHandle = g_pI2CIOMHandle;
        //
        // Return the status.
        //
        return AM_DEVICES_I2C_STATUS_SUCCESS;
    }
}

//*****************************************************************************
//
//! @brief De-Initialize the i2c driver.
//!
//! @param ui32Module     - IOM Module#
//!
//! This function reverses the initialization
//!
//! @return Status.
//
//*****************************************************************************
uint32_t
am_devices_i2c_term(uint32_t ui32Module)
{
    if ( ui32Module > AM_REG_IOM_NUM_MODULES )
    {
        return AM_DEVICES_I2C_STATUS_ERROR;
    }

    //
    // Disable the IOM.
    //
    am_hal_iom_disable(g_pI2CIOMHandle);

    //
    // Disable power to and uninitialize the IOM instance.
    //
    am_hal_iom_power_ctrl(g_pI2CIOMHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);

    am_hal_iom_uninitialize(g_pI2CIOMHandle);

    //
    // Return the status.
    //
    return AM_DEVICES_I2C_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Programs the given range of flash addresses.
//!
//! @param ui32DeviceNumber - Device number of the external flash
//! @param pui8TxBuffer - Buffer to write the external flash data from
//! @param ui32WriteAddress - Address to write to in the external flash
//! @param ui32NumBytes - Number of bytes to write to the external flash
//!
//! This function uses the data in the provided pui8TxBuffer and copies it to
//! the external flash at the address given by ui32WriteAddress. It will copy
//! exactly ui32NumBytes of data from the original pui8TxBuffer pointer. The
//! user is responsible for ensuring that they do not overflow the target flash
//! memory or underflow the pui8TxBuffer array
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_i2c_blocking_write(uint16_t ui16SadAddress,
                                     uint16_t ui16WriteAddress,
                                     uint8_t *pui8TxBuffer,
                                     uint16_t ui16NumBytes)
{
    //
    // Write the data to the device.
    //
    if (am_device_command_write(ui16SadAddress, 1,
                            (ui16WriteAddress),
                            false, (uint32_t *)pui8TxBuffer, ui16NumBytes))
    {
        return AM_DEVICES_I2C_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_I2C_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Programs the given range of flash addresses.
//!
//! @param ui32DeviceNumber - Device number of the external flash
//! @param pui8TxBuffer - Buffer to write the external flash data from
//! @param ui32WriteAddress - Address to write to in the external flash
//! @param ui32NumBytes - Number of bytes to write to the external flash
//!
//! This function uses the data in the provided pui8TxBuffer and copies it to
//! the external flash at the address given by ui32WriteAddress. It will copy
//! exactly ui32NumBytes of data from the original pui8TxBuffer pointer. The
//! user is responsible for ensuring that they do not overflow the target flash
//! memory or underflow the pui8TxBuffer array
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_i2c_nonblocking_write(uint16_t ui16SadAddress,
                                        uint16_t ui16WriteAddress,
                                        uint8_t *pui8TxBuffer,
                                        uint16_t ui16NumBytes,
                                        am_hal_iom_callback_t pfnCallback,
                                        void *pCallbackCtxt)
{
    am_hal_iom_transfer_t         Transaction;

    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.ui8Priority     = 1;        // High priority for now.
    Transaction.uPeerInfo.ui32I2CDevAddr = ui16SadAddress;
    Transaction.bContinue       = false;

    //
    // Set up the IOM transaction.
    //
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = ui16WriteAddress;
    Transaction.ui32NumBytes    = ui16NumBytes;
    Transaction.pui32TxBuffer   = (uint32_t *)pui8TxBuffer;

    //
    // Add this transaction to the command queue (no callback).
    //
    if (am_hal_iom_nonblocking_transfer(g_pI2CIOMHandle, &Transaction, pfnCallback, pCallbackCtxt))
    {
        return AM_DEVICES_I2C_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_I2C_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads the contents of the fram into a buffer.
//!
//! @param pui8RxBuffer - Buffer to store the received data from the flash
//! @param ui32ReadAddress - Address of desired data in external flash
//! @param ui32NumBytes - Number of bytes to read from external flash
//!
//! This function reads the external flash at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_i2c_blocking_read(uint16_t ui16SadAddress,
                                    uint16_t ui16ReadAddress,
                                    uint8_t *pui8RxBuffer,
                                    uint16_t ui16NumBytes)
{
    if (am_device_command_read(ui16SadAddress, 1,
                           (ui16ReadAddress),
                           false, (uint32_t *)pui8RxBuffer, ui16NumBytes))
    {
        return AM_DEVICES_I2C_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_I2C_STATUS_SUCCESS;
}


//*****************************************************************************
//
//! @brief Reads the contents of the fram into a buffer.
//!
//! @param pui8RxBuffer - Buffer to store the received data from the flash
//! @param ui32ReadAddress - Address of desired data in external flash
//! @param ui32NumBytes - Number of bytes to read from external flash
//!
//! This function reads the external flash at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_i2c_nonblocking_read(uint16_t ui16SadAddress,
                                       uint16_t ui16ReadAddress,
                                       uint8_t *pui8RxBuffer,
                                       uint16_t ui16NumBytes,
                                       am_hal_iom_callback_t pfnCallback,
                                       void * pCallbackCtxt)
{
    am_hal_iom_transfer_t Transaction;

    //
    // Set up the IOM transaction.
    //
    Transaction.ui8Priority     = 1;        // High priority for now.
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = (ui16ReadAddress);
    Transaction.ui32NumBytes    = ui16NumBytes;
    Transaction.pui32RxBuffer   = (uint32_t *)pui8RxBuffer;
    Transaction.uPeerInfo.ui32I2CDevAddr = ui16SadAddress;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.bContinue       = false;

    //
    // Start the transaction.
    //
    if (am_hal_iom_nonblocking_transfer(g_pI2CIOMHandle, &Transaction, pfnCallback, pCallbackCtxt))
    {
        return AM_DEVICES_I2C_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_I2C_STATUS_SUCCESS;
}
