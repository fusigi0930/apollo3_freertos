/*********************/
/*** Include Files ***/
/*********************/
#include <stdlib.h>
#include <string.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_devices_i2c.h"

/*********************/
/*** Macro Defines ***/
/*********************/
#define CLOCK_FREQ 1000  //It is clock frequency setting.
#define IOM_MODULE_NUMBER   0
#define IOM_IRQn            ((IRQn_Type)(IOMSTR0_IRQn + IOM_MODULE_NUMBER))

//
// Take over the interrupt handler for whichever IOM we're using.
//
#define iom_isr                                                          \
    am_iom_isr1(IOM_MODULE_NUMBER)
#define am_iom_isr1(n)                                                   \
    am_iom_isr(n)
#define am_iom_isr(n)                                                    \
    am_iomaster ## n ## _isr


/*************************/
/*** Variables Defines ***/
/*************************/
uint32_t  DMATCBBuffer[256];
void      *g_pIOMHandle;

am_hal_iom_config_t     g_sIomCfg =
{
    .eInterfaceMode       = AM_HAL_IOM_I2C_MODE,
    .ui32ClockFreq        = AM_HAL_IOM_100KHZ,
};

static uint32_t m_tickLast = 0;
static uint64_t m_elpsedTime_us = 0;


/************************/
/*** Function Defines ***/
/************************/
int CWM_OS_dbgPrintf(const char * format,...);


/********************************/
/*** Function Implementations ***/
/********************************/
void iom_isr(void)
{
    uint32_t ui32Status;

    if (!am_hal_iom_interrupt_status_get(g_pIOMHandle, true, &ui32Status))
    {
        if ( ui32Status )
        {
            am_hal_iom_interrupt_clear(g_pIOMHandle, ui32Status);
            am_hal_iom_interrupt_service(g_pIOMHandle, ui32Status);
        }
    }
}

static void I2C_init(void)
{
    g_sIomCfg.ui32NBTxnBufLength = sizeof(DMATCBBuffer) / 4;
    g_sIomCfg.pNBTxnBuf = &DMATCBBuffer[0];

    am_devices_i2c_init(IOM_MODULE_NUMBER, &g_sIomCfg, &g_pIOMHandle);

    return;
}

static void am_i2c_setting(void)
{
    I2C_init();
    NVIC_EnableIRQ(IOM_IRQn);
}

static void am_i2c_deinit(void)
{
    am_devices_i2c_term(0);
    NVIC_DisableIRQ(IOM_IRQn);
}

static void APOLLO_I2C_Reset(void)
{
    am_i2c_deinit();
    am_util_delay_ms(20);
    am_i2c_setting();
}

static void stimer_init(void)
{
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA);
    NVIC_EnableIRQ(STIMER_CMPR0_IRQn);
    am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
    am_hal_stimer_compare_delta_set(0, CLOCK_FREQ);//1 second timer
    am_hal_stimer_config(AM_HAL_STIMER_LFRC_1KHZ |
                         AM_HAL_STIMER_CFG_COMPARE_A_ENABLE);
    m_tickLast = am_hal_stimer_counter_get();
}

static uint64_t platsystickTicksToUs(uint64_t tick) {
    return (uint64_t)(tick * 1000000/CLOCK_FREQ);
}

static inline uint64_t calcElapsedTicks(uint64_t tickNow, uint64_t tickLast)
{
    if(tickNow >= tickLast)
        return tickNow - tickLast;
    else
        return 0xFFFFFFFF - tickLast + tickNow;
}

static void systickUpdate(void)
{
    //To implement the CWM_OS_GetTimeNs(), this function needs to be triggered every second
    uint64_t tickNow;
    uint64_t delta_ticks;

    tickNow = am_hal_stimer_counter_get();
    delta_ticks = calcElapsedTicks(tickNow, m_tickLast);
    m_elpsedTime_us += platsystickTicksToUs(delta_ticks);

    m_tickLast = tickNow;
}

#if 0
void am_stimer_cmpr0_isr(void)
{
    //To implement the CWM_OS_GetTimeNs(), this function needs to be triggered every second
    systickUpdate();

    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
    am_hal_stimer_compare_delta_set(0, CLOCK_FREQ);//1 second timer
}
#endif

#ifdef REBURN_PROCESS
static uint32_t mINFO0[] = {
    0x48EAAD88, //0000
    0xC9705737, //0004
    0x0A6B8458, //0008
    0xE41A9D74, //000C
    0x00055FFF, //0010
    0xFFFFFFFF, //0014
    0xFFFFFFFF, //0018
    0xFFFFFFFF, //001C
    0x0000007F, //0020
    0x13884021, //0024
    0x01C200C0, //0028
    0xFFFF1617, //002C
    0x00000002, //0030
    0x00000000, //0034
    0x00000000, //0038
    0x00000000, //003C
    0x00000000, //0040
    0xFFFFFFFF, //0044
    0xFFFFFFFF, //0048
    0xFFFFFFFF, //004C
    0x00000000, //0050
    0x0000C000, //0C00
    0x00000000, //1800
    0x00000000, //1804
    0x00000000, //1808
    0x00000000, //180C
    0xFFFFFFFF, //1810
    0xFFFFFFFF, //1814
    0xFFFFFFFF, //1818
    0xFFFFFFFF, //181C
    0xAAAAAAAA, //1820
    0xAAAAAAAA, //1824
    0xAAAAAAAA, //1828
    0xAAAAAAAA, //182C
    0x55555555, //1830
    0x55555555, //1834
    0x55555555, //1838
    0x55555555, //183C
    0x11111111, //1840
    0x11111111, //1844
    0x11111111, //1848
    0x11111111, //184C
    0xA5A5A5A5, //1850
    0xA5A5A5A5, //1854
    0xA5A5A5A5, //1858
    0xA5A5A5A5, //185C
    0x66666666, //1860
    0x66666666, //1864
    0x66666666, //1868
    0x66666666, //186C
    0xDEADBEEF, //1870
    0xDEADBEEF, //1874
    0xDEADBEEF, //1878
    0xDEADBEEF, //187C
    0x00000000, //1880
    0x00000000, //1884
    0x00000000, //1888
    0x00000000, //188C
    0x00000000, //1890
    0x00000000, //1894
    0x00000000, //1898
    0x00000000, //189C
    0xFFFFFFFF, //18A0
    0xFFFFFFFF, //18A4
    0xFFFFFFFF, //18A8
    0xFFFFFFFF, //18AC
    0xFFFFFFFF, //18B0
    0xFFFFFFFF, //18B4
    0xFFFFFFFF, //18B8
    0xFFFFFFFF, //18BC
    0x55AA55AA, //18C0
    0x55AA55AA, //18C4
    0x55AA55AA, //18C8
    0x55AA55AA, //18CC
    0x55AA55AA, //18D0
    0x55AA55AA, //18D4
    0x55AA55AA, //18D8
    0x55AA55AA, //18DC
    0xDEADBEEF, //18E0
    0xDEADBEEF, //18E4
    0xDEADBEEF, //18E8
    0xDEADBEEF, //18EC
    0xDEADBEEF, //18F0
    0xDEADBEEF, //18F4
    0xDEADBEEF, //18F8
    0xDEADBEEF, //18FC
    0xDEADBEEF, //1A00
    0xDEADBEEF, //1A04
    0xDEADBEEF, //1A08
    0xDEADBEEF, //1A0C
};

#define UNIT_DWORD    (4)
#define ADDR_BASE     (AM_HAL_FLASH_INFO_ADDR)

//Return Codes
#define APOLLO3_WRITE_RET_OK      (0)
#define APOLLO3_WRITE_RET_FAIL    (1)

/*
 * DESCRIPTION of apollo3_info0_write()
 * Argument [address] is absolute address in memory map, its unit is Byte
 * Local variable [offset_in_dword] is relative address start from 0x50020000, its unit is DWORD(4 Bytes)
 * apollo3_info0_write() would translate address into offset_in_dword to fit the Ambiq SDK requirement
 */
static int apollo3_info0_write(uint32_t offset_in_dword, uint32_t * ui32_buf, uint32_t length)
{
    static int erased = 0;
    uint8_t am_ret = 0;
    uint32_t length_in_dword;

    if(length % UNIT_DWORD != 0) {
        CWM_OS_dbgPrintf("[info0 write]length not align to 4 bytes (%d)\n", length);
        goto exit;
    }
    length_in_dword = length / UNIT_DWORD;

    if (!erased) {
        am_ret = am_hal_flash_erase_info(AM_HAL_FLASH_INFO_KEY, 0);
        if(am_ret) {
            CWM_OS_dbgPrintf("[info0 write]erase failed\n");
            goto exit;
        }
        CWM_OS_dbgPrintf("[info0 write]erase info0 area success\n");
        erased = 1;
    }

    am_ret = am_hal_flash_program_info(AM_HAL_FLASH_INFO_KEY, 0, ui32_buf, offset_in_dword, length_in_dword);
    if (am_ret) {
        CWM_OS_dbgPrintf("[info0 write]write failed\n");
        goto exit;
    }
    return APOLLO3_WRITE_RET_OK;
exit:
    return APOLLO3_WRITE_RET_FAIL;
}

static int _apollo3_rebuild_info0(uint32_t address, uint32_t* ui32_buf, uint32_t length)
{
    int rc;
    uint32_t offset_in_dword;

    rc = apollo3_info0_write(0, &mINFO0[0], 21 * UNIT_DWORD);
    if(rc != APOLLO3_WRITE_RET_OK) {
        goto exit;
    }
    rc = apollo3_info0_write(0x300, &mINFO0[21], 1 * UNIT_DWORD);
    if(rc != APOLLO3_WRITE_RET_OK) {
        goto exit;
    }
    rc = apollo3_info0_write(0x600, &mINFO0[22], 64 * UNIT_DWORD);
    if(rc != APOLLO3_WRITE_RET_OK) {
        goto exit;
    }
    rc = apollo3_info0_write(0x680, &mINFO0[86], 4 * UNIT_DWORD);
    if(rc != APOLLO3_WRITE_RET_OK) {
        goto exit;
    }
    CWM_OS_dbgPrintf("[info0]bootloader write success\n");

    if(address < ADDR_BASE) {
        CWM_OS_dbgPrintf("[info0 write]address < 0x%08x (0x%08x)\n", ADDR_BASE, address);
        goto exit;
    }
    if(address % UNIT_DWORD != 0) {
        CWM_OS_dbgPrintf("[info0]address not aligned to 4 bytes (%08x)\n", address);
        goto exit;
    }
    offset_in_dword = (address - ADDR_BASE) / UNIT_DWORD;
    rc = apollo3_info0_write(offset_in_dword, ui32_buf, length);
    if(rc != APOLLO3_WRITE_RET_OK) {
        CWM_OS_dbgPrintf("[info0]security write fail\n");
        goto exit;
    }
    CWM_OS_dbgPrintf("[info0]security write success\n");
    /**********************************************************************
     * If Apollo3 Info0 area is totally empty, a "cold boot" is needed
     * after we update info0 data. Before we "cold boot"
     * the device, we cannot read the info0 data that we just wrote into.
     * Please remember that.
     **********************************************************************/
    CWM_OS_dbgPrintf("[info0]please remove battery or power cable from device, wait few 1-2 seconds then re-connect it again\n");
    return APOLLO3_WRITE_RET_OK;
exit:
    return APOLLO3_WRITE_RET_FAIL;
}

//AM_HAL_FLASH_INFO_ADDR = 0x50020000 defined @ AMBIQ_SDK/mcu/apollo3/hal/am_hal_flash.h
#define APOLLO3_INFO0_SIZE                  (16 * 1024)
#define APOLLO3_INFO0_LARGEST_VALID_ADDR    (AM_HAL_FLASH_INFO_ADDR + APOLLO3_INFO0_SIZE - 1)
int CWM_OS_flashWrite(uint32_t address, uint8_t *ui8_buf,  uint32_t len)
{
    int rc;

    CWM_OS_dbgPrintf("[OS_flashWrite]write addr(0x%08x)  len(%d)\n", address, len);
    if(address >= AM_HAL_FLASH_INFO_ADDR && address <= APOLLO3_INFO0_LARGEST_VALID_ADDR) {
        rc = _apollo3_rebuild_info0(address, (uint32_t *)ui8_buf, len);
        if(rc != APOLLO3_WRITE_RET_OK) {
            goto error_exit;
        }
    } else {
        CWM_OS_dbgPrintf("[OS_flashWrite]invalid write address 0x%08x\n", address);
        goto error_exit;
    }
    return APOLLO3_WRITE_RET_OK;
error_exit:
    return APOLLO3_WRITE_RET_FAIL;
}
#endif

#define BUS_RETRY_TIMES     3
#define BUS_RECOVER_TIMES   1

int CWM_OS_i2cRead(uint16_t slaveAddr, uint16_t reg, int regLength, uint8_t *readData, int readDataSize, int argument)
{
    int res;
    int rty = BUS_RETRY_TIMES;
    int rcovery = BUS_RECOVER_TIMES;

    do{
        res = am_devices_i2c_blocking_read(slaveAddr, reg, readData, readDataSize);

        if(res == 0)
            break;

        rty --;
        if(rcovery > 0)
        {
            if(rty == 0)
            {
                rty = BUS_RETRY_TIMES;
                rcovery --;
                APOLLO_I2C_Reset();
            }
        }
    }while(rty > 0);


    return (res>0) ? res*-1: res;
}

int CWM_OS_i2cWrite(uint16_t slaveAddr, uint16_t reg, int regLength, uint8_t *writeData, int writeDataSize, int argument)
{
    int res;
    int rty = BUS_RETRY_TIMES;
    int rcovery = BUS_RECOVER_TIMES;

    do{
        res = am_devices_i2c_blocking_write(slaveAddr, reg, writeData, writeDataSize);

        if(res == 0)
            break;

        rty --;
        if(rcovery > 0)
        {
            if(rty == 0)
            {
                rty = BUS_RETRY_TIMES;
                rcovery --;
                APOLLO_I2C_Reset();
            }
        }
    }while(rty > 0);

    return (res>0) ? res*-1: res;
}

int CWM_OS_i2cTransfer(uint16_t slaveAddr, uint16_t reg, int regLength, uint8_t *writeData, int writeDataSize, uint8_t *readData, int readDataSize, int argument)
{
    am_hal_iom_transfer_t desc;
    memset(&desc, 0, sizeof(am_hal_iom_transfer_t));

    desc.ui32InstrLen    = regLength;
    desc.ui32Instr       = reg;
    desc.uPeerInfo.ui32I2CDevAddr = slaveAddr >> 1;
    desc.ui8RepeatCount  = 0;
    desc.ui32PauseCondition = 0;
    desc.ui32StatusSetClr = 0;
    desc.bContinue = false;

    if (readDataSize == 0) {
        if (writeDataSize == 0 || writeData == NULL)
            return -1;

        // write mode
        desc.eDirection      = AM_HAL_IOM_TX;
        desc.ui32NumBytes    = writeDataSize;
        desc.pui32TxBuffer   = (uint32_t *) writeData;
    }
    else {
        if (readData == NULL)
            return -1;

        // read mode
        desc.eDirection      = AM_HAL_IOM_RX;
        desc.ui32NumBytes    = readDataSize;
        desc.pui32RxBuffer   = (uint32_t *) readData;
    }

    return am_hal_iom_blocking_transfer(g_pIOMHandle, &desc);
}

void CWM_OS_uSleep(uint32_t time)
{
    uint32_t ms = time / 1000;

    if (ms == 0)
        ms = 1;
    am_util_delay_ms(ms);
}

uint64_t CWM_OS_GetTimeNs(void)
{
    uint64_t tickNow;
    uint64_t delta_ticks;
    uint64_t timeNow_us;

    tickNow = am_hal_stimer_counter_get();
    delta_ticks = calcElapsedTicks(tickNow, m_tickLast);
    timeNow_us = m_elpsedTime_us + platsystickTicksToUs(delta_ticks);

    return timeNow_us * 1000;
}

void* CWM_OS_malloc(int size)
{
    if (size == 0)
        size = 1;
    return malloc(size);
}

void CWM_OS_free(void *ptr)
{
    if (ptr == NULL)
        return;

    free(ptr);
}

int CWM_OS_dbgOutput(const char * format)
{
    am_util_stdio_printf("%s", format);
    return 0;
}

int CWM_OS_dbgPrintf(const char * format,...)
{
    char InputBuff[255];
    va_list    args;

    va_start(args, format);
    am_util_stdio_vsprintf(InputBuff, format, args);
    va_end(args);
    CWM_OS_dbgOutput(InputBuff);
    return 0;
}

void board_init(void)
{
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);  //Set the clock frequency.
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);  //default cache configuration
    am_hal_cachectrl_enable();
    am_bsp_low_power_init();  //Configure the board for low power operation.

#ifdef PRINTF_FROM_UART
    am_bsp_uart_printf_enable();  //Enable the UART print interface.
#else
    am_bsp_itm_printf_enable();  //Enable the ITM print interface. Using "SWO Viewer" to show log
#endif
    am_util_stdio_terminal_clear();  //Clear the terminal and print the banner.
    am_i2c_setting();
    am_hal_interrupt_master_enable();

    stimer_init();  //Timer Init
}

