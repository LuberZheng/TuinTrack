/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ====================== CC2650RC.c =========================================
 *  This file is responsible for setting up the board specific items for the
 *  CC2650 remote controle.
 */

/*
 *  ====================== Includes ============================================
 */
#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <driverlib/udma.h>
#include <driverlib/osc.h>
#include <driverlib/cpu.h>
#include <driverlib/pwr_ctrl.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/aon_event.h>
#include <driverlib/ioc.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/dma/UDMACC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

#include "Board.h"
/*
 *  ========================= IO driver initialization =========================
 *  From main, PIN_init(BoardGpioInitTable) should be called to setup safe
 *  settings for this board.
 *  When a pin is allocated and then de-allocated, it will revert to the state
 *  configured in this table.
*/

PIN_Config BoardGpioInitTable[] = {

    Board_LED_G       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off            */
    Board_LED_R       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off            */
    Board_LED_IR      | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,     /* LED initially off            */
    Board_BUZZER             | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW       | PIN_PUSHPULL    | PIN_DRVSTR_MAX, /* LED initially off           */
#ifdef APP_DEBUG_PINS
    Board_DEBUG_APP   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif //APP_DEBUG_PINS
#ifdef  IRGENCC26XX_DEBUG
    Board_IR_DATA_CH_DEBUG   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_IR_SHADOW_CH_DEBUG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_IR_OUTPUT_DEBUG    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif //IRGENCC26XX_DEBUG
    Board_MPU_INT            | PIN_INPUT_EN       | PIN_PULLDOWN       | PIN_IRQ_NEGEDGE | PIN_HYSTERESIS, /* MPU_INT is active low        */
    Board_MPU_POWER          | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH      | PIN_PUSHPULL    | PIN_DRVSTR_MAX, /* MPU initially on             */
    Board_MIC_POWER          | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW       | PIN_PUSHPULL    | PIN_DRVSTR_MIN, /* MIC initially off            */
    Board_SPI_FLASH_CS       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH      | PIN_PUSHPULL    | PIN_DRVSTR_MIN, /* External flash chip select   */
    Board_SPI_DEVPK_CS       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW       | PIN_PUSHPULL    | PIN_DRVSTR_MIN, /* DevPack chip select          */
    Board_KEY_COL1           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW       | PIN_PUSHPULL    | PIN_DRVSTR_MAX, /* Col 1 is active low          */
    Board_KEY_COL2           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW       | PIN_PUSHPULL    | PIN_DRVSTR_MAX, /* Col 2 is active low          */
    Board_KEY_COL3           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW       | PIN_PUSHPULL    | PIN_DRVSTR_MAX, /* Col 3 is active low          */
    Board_KEY_PWR            | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW       | PIN_PUSHPULL    | PIN_DRVSTR_MAX, /* KEY PWR initially off        */
    Board_KEY_CLK            | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW       | PIN_PUSHPULL    | PIN_DRVSTR_MAX, /* KEY CLK initially off        */
    Board_KEY_SCAN           | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW       | PIN_PUSHPULL    | PIN_DRVSTR_MAX, /* KEY SCAN initially low       */
    Board_I2C0_SDA1          | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW       | PIN_PUSHPULL    | PIN_DRVSTR_MAX,
    Board_UART_RX            | PIN_INPUT_EN       | PIN_PULLDOWN,                                          /* DevPack */
    Board_UART_TX            | PIN_INPUT_DIS      | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH   | PIN_PUSHPULL,   /* UART TX pin at inactive level */
    Board_SPI0_MOSI          | PIN_INPUT_EN       | PIN_PULLDOWN,                                            /* SPI master out - slave in */
    Board_SPI0_MISO          | PIN_INPUT_EN       | PIN_PULLDOWN,                                            /* SPI master in - slave out */
    Board_SPI0_CLK           | PIN_INPUT_EN       | PIN_PULLDOWN,                                             /* SPI clock */
    Board_DEVPK_ID           | PIN_INPUT_EN       | PIN_NOPULL,                                            /* Device pack ID - external PU  */
    PIN_TERMINATE                                                                              /* Terminate list               */
};

const PINCC26XX_HWAttrs PINCC26XX_hwAttrs = {
    .intPriority = ~0,
    .swiPriority = 0
};

/*
 *  ========================== Crypto begin =======================================
 *  NOTE: The Crypto implementaion should be considered experimental and not validated!
*/
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(CryptoCC26XX_config, ".const:CryptoCC26XX_config")
#pragma DATA_SECTION(cryptoCC26XXHWAttrs, ".const:cryptoCC26XXHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/crypto/CryptoCC26XX.h>

/* Crypto objects */
CryptoCC26XX_Object cryptoCC26XXObjects[CC2650_CRYPTOCOUNT];

/* Crypto configuration structure, describing which pins are to be used */
const CryptoCC26XX_HWAttrs cryptoCC26XXHWAttrs[CC2650_CRYPTOCOUNT] = {
    {
        .baseAddr = CRYPTO_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_CRYPTO,
        .intNum = INT_CRYPTO_RESULT_AVAIL_IRQ,
        .intPriority = ~0,
    }
};


/* Crypto configuration structure */
const CryptoCC26XX_Config CryptoCC26XX_config[] = {
    {
        .object = &cryptoCC26XXObjects[0],
        .hwAttrs = &cryptoCC26XXHWAttrs[0]
    },
    {NULL, NULL}
};
/*
 *  ========================== Crypto end =========================================
*/

/*
 *  ============================= IRGEN begin ===================================
*/
#ifdef TI_DRIVERS_IRGEN_INCLUDED
#include "IRGENCC26XX.h"
IRGENCC26XX_Object irgenCC26XXObject = {0};

const IRGENCC26XX_HWAttrs irgenCC26XXHWAttrs = {

     .irLedPin              = Board_LED_IR,
#ifdef IRGENCC26XX_DEBUG
     .irOutputPin           = Board_IR_OUTPUT_DEBUG,
     .irDataChPin           = Board_IR_DATA_CH_DEBUG,
     .irShadowChPin         = Board_IR_SHADOW_CH_DEBUG,
#endif //IRGENCC26XX_DEBUG
     .dmaChannelBitMask     = (( 1 << UDMA_CHAN_TIMER0_A) | ( 1 << UDMA_CHAN_TIMER0_B)  | ( 1 << UDMA_CHAN_TIMER1_A)  |  \
                               ( 1 << UDMA_CHAN_SW_EVT0 ) | ( 1 << UDMA_CHAN_SW_EVT1 )  | ( 1 << UDMA_CHAN_SW_EVT2 )  |  \
                               ( 1 << UDMA_CHAN_SW_EVT3 )),
     .dmaSoftChannelBitMask = (( 1 << UDMA_CHAN_SW_EVT0 ) | ( 1 << UDMA_CHAN_SW_EVT1 ) | \
                               ( 1 << UDMA_CHAN_SW_EVT2 ) | ( 1 << UDMA_CHAN_SW_EVT3 )),
};

/* IRGEN configuration structure */
const IRGENCC26XX_Config IRGENCC26XX_config = {
    &irgenCC26XXObject,
    &irgenCC26XXHWAttrs
};
#endif /* TI_DRIVERS_IRGEN_INCLUDED */
/*
 *  ============================= IRGEN end ===================================
*/

/*
 *  ============================= UDMA begin ===================================
*/
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UDMACC26XX_config, ".const:UDMACC26XX_config")
#pragma DATA_SECTION(udmaHWAttrs, ".const:udmaHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/dma/UDMACC26XX.h>

/* UDMA objects */
UDMACC26XX_Object UdmaObjects[CC2650_UDMACOUNT];

/* UDMA configuration structure */
const UDMACC26XX_HWAttrs udmaHWAttrs[CC2650_UDMACOUNT] = {
    {
      .baseAddr = UDMA0_BASE,
      .powerMngrId = PowerCC26XX_PERIPH_UDMA,
      .intNum = INT_DMA_ERR,
      .intPriority = ~0
    }
};

/* UDMA configuration structure */
const UDMACC26XX_Config UDMACC26XX_config[] = {
    {
        .object = &UdmaObjects[0],
        .hwAttrs = &udmaHWAttrs[0]
    },
    {NULL, NULL},
};
/*
 *  ============================= UDMA end =====================================
*/

/*
 *  ============================= UART begin ===================================
*/
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UART_config, ".const:UART_config")
#pragma DATA_SECTION(uartCC26XXHWAttrs, ".const:uartCC26XXHWAttrs")
#endif
/* Include drivers */

#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

/* UART objects */
UARTCC26XX_Object uartCC26XXObjects[CC2650_UARTCOUNT];

/* UART hardware parameter structure, also used to assign UART pins */
const UARTCC26XX_HWAttrsV1 uartCC26XXHWAttrs[CC2650_UARTCOUNT] = {
    {    /* CC2650_UART0 */
        .baseAddr = UART0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_UART0,
        .intNum = INT_UART0_COMB,
        .intPriority = ~0,
        .txPin = Board_UART_TX,
        .rxPin = Board_UART_RX,
        .ctsPin = PIN_UNASSIGNED,
        .rtsPin = PIN_UNASSIGNED
    }
};

/* UART configuration structure */
const UART_Config UART_config[] = {
    {
        .fxnTablePtr = &UARTCC26XX_fxnTable,
        .object = &uartCC26XXObjects[0],
        .hwAttrs = &uartCC26XXHWAttrs[0]
    },
    { NULL, NULL, NULL }
};
/*
 *  ============================= UART end =====================================
*/

/*
 *  ============================= PDM begin ===================================
*/
#include <ti/drivers/pdm/PDMCC26XX.h>
PDMCC26XX_Object pdmCC26XXObject = {0};

const PDMCC26XX_HWAttrs pdmCC26XXHWAttrs = {
     .micPower = Board_MIC_POWER,
     .taskPriority = 2,
};

/* PDM configuration structure */
const PDMCC26XX_Config PDMCC26XX_config[] = {
    {
        .object = &pdmCC26XXObject,
        .hwAttrs = &pdmCC26XXHWAttrs
    },
    {NULL, NULL}
};

/*
 *  ========================= RF driver begin ==================================
 */
/* Include drivers */
#include <ti/drivers/rf/RF.h>

/* RF hwi and swi priority */
const RFCC26XX_HWAttrs RFCC26XX_hwAttrs = {
    .hwiCpe0Priority = ~0,
    .hwiHwPriority   = ~0,
    .swiCpe0Priority =  5,
    .swiHwPriority   =  5,
};

/*
 *  ========================== RF driver end ===================================
 */
/*
 *  ============================= UART end =====================================
*/

/*
 *  ============================= I2S begin =====================================
*/
#include <ti/drivers/pdm/PDMCC26XX_util.h>

PDMCC26XX_I2S_Object i2sCC26XXObject;

const PDMCC26XX_I2S_HWAttrs i2sCC26XXHWAttrs = {
    .baseAddr = I2S0_BASE,
    .intNum = INT_I2S_IRQ,
    .intPriority = ~0,
    .powerMngrId = PowerCC26XX_PERIPH_I2S,
    .mclkPin = Board_I2S_MCLK,
    .bclkPin = Board_I2S_BCLK,
    .wclkPin = Board_I2S_WCLK,
    .ad0Pin = Board_I2S_ADI,
};

/* I2S configuration structure */
const PDMCC26XX_I2S_Config PDMCC26XX_I2S_config[] = {
    {
        .object = &i2sCC26XXObject,
        .hwAttrs = &i2sCC26XXHWAttrs
    },
    {NULL, NULL}
};

/*
 *  ============================= I2S end =====================================
*/

/*
 *  ============================= I2C Begin=========================================
*/
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(I2C_config, ".const:I2C_config")
#pragma DATA_SECTION(i2cCC26xxHWAttrs, ".const:i2cCC26xxHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/i2c/I2CCC26XX.h>

/* I2C objects */
I2CCC26XX_Object i2cCC26xxObjects[CC2650_I2CCOUNT];

/* I2C configuration structure, describing which pins are to be used */
const I2CCC26XX_HWAttrsV1 i2cCC26xxHWAttrs[CC2650_I2CCOUNT] = {
    {
        .baseAddr = I2C0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_I2C0,
        .intNum = INT_I2C_IRQ,
        .intPriority = ~0,
        .sdaPin = Board_I2C0_SDA1,
        .sclPin = Board_I2C0_SCL1,
    }
};

const I2C_Config I2C_config[] = {
    {
        .fxnTablePtr = &I2CCC26XX_fxnTable,
        .object = &i2cCC26xxObjects[0],
        .hwAttrs = &i2cCC26xxHWAttrs[0]
    },
    {NULL, NULL, NULL}
};
/*
 *  ========================== I2C end =====================================
*/


/*
 *  ========================== SPI DMA begin ===================================
*/
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(SPI_config, ".const:SPI_config")
#pragma DATA_SECTION(spiCC26XXDMAHWAttrs, ".const:spiCC26XXDMAHWAttrs")
#endif /* __TI_COMPILER_VERSION__ */

/* Include drivers */
#include <ti/drivers/spi/SPICC26XXDMA.h>

/* SPI objects */
SPICC26XXDMA_Object spiCC26XXDMAObjects[CC2650_SPICOUNT];

/* SPI configuration structure, describing which pins are to be used */
const SPICC26XXDMA_HWAttrsV1 spiCC26XXDMAHWAttrs[CC2650_SPICOUNT] = {
    {   /* SENSORTAG_CC2650_SPI0 */
        .baseAddr = SSI0_BASE,
        .intNum = INT_SSI0_COMB,
        .intPriority = ~0,
        .defaultTxBufValue = 0,
        .powerMngrId = PowerCC26XX_PERIPH_SSI0,
        .rxChannelBitMask = 1<<UDMA_CHAN_SSI0_RX,
        .txChannelBitMask = 1<<UDMA_CHAN_SSI0_TX,
        .mosiPin = Board_SPI0_MOSI,
        .misoPin = Board_SPI0_MISO,
        .clkPin = Board_SPI0_CLK,
        .csnPin = Board_SPI0_CSN
    }
};

/* SPI configuration structure */
const SPI_Config SPI_config[] = {
    /* SENSORTAG_CC2650_SPI0 */
    {
        .fxnTablePtr = &SPICC26XXDMA_fxnTable,
        .object = &spiCC26XXDMAObjects[0],
        .hwAttrs = &spiCC26XXDMAHWAttrs[0]
    },
    {NULL, NULL, NULL},
};
/*
 *  ========================== SPI DMA end =====================================
*/

/*
 *  ============================= Watchdog begin =====================================
*/
#ifdef TI_DRIVERS_WATCHDOG_INCLUDED
#include "WatchdogCC26XX.h"
WatchdogCC26XX_Object wdCC26XXObject;

const WatchdogCC26XX_HWAttrs wdCC26XXHWAttrs = {
    WDT_BASE,
    INT_WDT_IRQ,
};

/* I2S configuration structure */
const Watchdog_Config Watchdog_config[] = {
    {
        .fxnTablePtr = &WatchdogCC26XX_fxnTable,
        .object = &wdCC26XXObject,
        .hwAttrs = &wdCC26XXHWAttrs
    },
    {NULL, NULL, NULL},
};
#endif //TI_DRIVERS_WATCHDOG_INCLUDED
/*
 *  ============================= Watchdog end =====================================
*/

/*
 *  ============================= Power begin ===================================
*/
const PowerCC26XX_Config PowerCC26XX_config = {
    .policyInitFxn = NULL,
    .policyFxn = &PowerCC26XX_standbyPolicy,
    .calibrateFxn = &PowerCC26XX_calibrate,
    .enablePolicy = TRUE,
    .calibrateRCOSC_LF = TRUE,
    .calibrateRCOSC_HF = TRUE,
};
/*
 *  ============================= Power end ===================================
*/

/*============================================================================*/
