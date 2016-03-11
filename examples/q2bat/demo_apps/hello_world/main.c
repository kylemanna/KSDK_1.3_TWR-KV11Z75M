/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

///////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////
// SDK Included Files
#include "board.h"
#include "fsl_lptmr_driver.h"
#include "fsl_debug_console.h"
#include "fsl_dac_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_flexcan_driver.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
// Timer period: 500000uS
#define TMR_PERIOD         500000U
#if defined(TWR_KV46F150M)
#define LPTMR0_IDX LPTMR_IDX
#endif

#define DAC_INSTANCE          BOARD_DAC_DEMO_DAC_INSTANCE

/* Global variables*/
uint32_t txIdentifier;
uint32_t rxIdentifier;
uint32_t txRemoteIdentifier;
uint32_t rxRemoteIdentifier;
uint32_t txMailboxNum;
uint32_t rxMailboxNum;
uint32_t rxRemoteMailboxNum;
uint32_t rxRemoteMailboxNum;
flexcan_state_t canState;

uint32_t instance = BOARD_CAN_INSTANCE;
uint32_t numErrors;

/* The following tables are the CAN bit timing parameters that are calculated by using the method
 * outlined in AN1798, section 4.1.
 */

/*
 * The table contains propseg, pseg1, pseg2, pre_divider, and rjw. The values are calculated for
 * a protocol engine clock of 60MHz
 */
flexcan_time_segment_t bitRateTable60Mhz[] = {
    { 6, 7, 7, 19, 3},  /* 125 kHz */
    { 6, 7, 7,  9, 3},  /* 250 kHz */
    { 6, 7, 7,  4, 3},  /* 500 kHz */
    { 6, 5, 5,  3, 3},  /* 750 kHz */
    { 6, 5, 5,  2, 3},  /* 1   MHz */
};
/*
 * The table contains propseg, pseg1, pseg2, pre_divider, and rjw. The values are calculated for
 * a protocol engine clock of 75MHz
 */
flexcan_time_segment_t bitRateTable75Mhz[] = {
    { 6, 7, 7, 25, 3},  /* 125 kHz */
    { 6, 7, 7, 12, 3},  /* 250 kHz */
    { 6, 6, 6,  6, 3},  /* 500 kHz */
    { 6, 4, 4,  5, 3},  /* 750 kHz */
    { 6, 3, 3,  4, 3},  /* 1   MHz */
};

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

void send_data(void)
{
    uint8_t data[8];
    uint32_t result, i;
    flexcan_data_info_t txInfo;

    /*Standard ID*/
    txInfo.msg_id_type = kFlexCanMsgIdStd;
    txInfo.data_length = 8;

    for (i = 0; i < 8; i++)
    {
        data[i] = 10 + i;
    }

    PRINTF("\r\nFlexCAN send config");
    result = FLEXCAN_DRV_ConfigTxMb(instance, txMailboxNum, &txInfo, txIdentifier);
    if (result)
    {
        PRINTF("\r\nTransmit MB config error. Error Code: 0x%lx", result);
    }
    else
    {
        result = FLEXCAN_DRV_SendBlocking(instance, txMailboxNum, &txInfo, txIdentifier,
                                  data, OSA_WAIT_FOREVER);
        if (result)
        {
            numErrors++;
            PRINTF("\r\nTransmit send configuration failed. result: 0x%lx", result);
        }
        else
        {
            PRINTF("\r\nData transmit: ");
            for (i = 0; i < txInfo.data_length; i++ )
            {
                PRINTF("%02x ", data[i]);
            }
        }
    }
}


/*!
 * @brief LPTMR interrupt call back function.
 * The function is used to toggle LED1.
 */
void lptmr_call_back(void)
{
    // Toggle LED1
    LED1_TOGGLE;
    LED2_TOGGLE;

    // Spam the UART
    PRINTF("\r\ntimer\r\n");
}

/*!
 * @brief Main function
 */
int main (void)
{
    // RX buffers
    //! @param receiveBuff Buffer used to hold received data
    uint8_t receiveBuff;

    // LPTMR configurations
    lptmr_user_config_t lptmrConfig =
    {
        .timerMode = kLptmrTimerModeTimeCounter,
        .freeRunningEnable = false,
        .prescalerEnable = true,
        .prescalerClockSource = kClockLptmrSrcLpoClk,
        .prescalerValue = kLptmrPrescalerDivide2,
        .isInterruptEnabled = true,
    };
    // LPTMR driver state information
    lptmr_state_t lptmrState;

    // Initialize standard SDK demo application pins
    hardware_init();

    // Initialize LPTMR
    LPTMR_DRV_Init(LPTMR0_IDX, &lptmrState, &lptmrConfig);
    // Set timer period for TMR_PERIOD seconds
    LPTMR_DRV_SetTimerPeriodUs(LPTMR0_IDX, TMR_PERIOD);
    // Install interrupt call back function for LPTMR
    LPTMR_DRV_InstallCallback(LPTMR0_IDX, lptmr_call_back);
    // Start LPTMR
    LPTMR_DRV_Start(LPTMR0_IDX);

    // Initialize LED1
    LED1_EN;
    LED2_EN;
    LED2_TOGGLE;

    // Turn CAN transceiver on
    GPIO_DRV_WritePinOutput(GPIO_MAKE_PIN(GPIOC_IDX, 5U), 0);

    // Print the initial banner
    PRINTF("\r\nHello World!\r\n\r\n");

    dac_converter_config_t dacUserConfig;
    // Fill the structure with configuration of software trigger.
    DAC_DRV_StructInitUserConfigNormal(&dacUserConfig);
    // Initialize the DAC Converter.
    DAC_DRV_Init(DAC_INSTANCE, &dacUserConfig);
    DAC_DRV_Output(DAC_INSTANCE, (uint16_t)0xff);

#if 0
    // Leave disabled by default as the output pin is in use for battery
    // balancing and will discharge the battery
    //
    // CLKOUT
    PORT_HAL_SetDriveStrengthMode(PORTC,3u,kPortLowDriveStrength);
    PORT_HAL_SetMuxMode(PORTC,3u,kPortMuxAlt5);
    PORT_HAL_SetSlewRateMode(PORTC,3u,kPortFastSlewRate);

    //SIM_WR_SOPT2_CLKOUTSEL(SIM, 0x2); // BUSCLK
    //SIM_WR_SOPT2_CLKOUTSEL(SIM, 0x3); // LPO 1kHz
    //SIM_WR_SOPT2_CLKOUTSEL(SIM, 0x4); // MCGIRCLK
    SIM_WR_SOPT2_CLKOUTSEL(SIM, 0x6); // OSCERCLK
#endif

    /*
     * Test CAN
     */
    uint32_t result;
    flexcan_user_config_t flexcanData;
    uint32_t canPeClk;
    numErrors = 0;

    flexcanData.max_num_mb = 16;
    flexcanData.num_id_filters = kFlexCanRxFifoIDFilters_8;
    flexcanData.is_rx_fifo_needed = false;
    //flexcanData.flexcanMode = kFlexCanLoopBackMode;
    flexcanData.flexcanMode = kFlexCanNormalMode;

    // Select mailbox number
    rxMailboxNum = 8;
    txMailboxNum = 9;
    rxRemoteMailboxNum = 10;
    rxRemoteMailboxNum = 11;

    // Select mailbox ID
    rxRemoteIdentifier = 0x0F0;
    txRemoteIdentifier = 0x00F;

    // Set rxIdentifier as same as txIdentifier to receive loopback data
    rxIdentifier = 0x123;
    txIdentifier = 0x123;
    result = FLEXCAN_DRV_Init(instance, &canState, &flexcanData);
    if (result)
    {
        numErrors++;
        PRINTF("\r\nFLEXCAN initilization. result: 0x%lx", result);
    }

    if (FLEXCAN_HAL_GetClock((g_flexcanBase[instance])))
    {
        canPeClk = CLOCK_SYS_GetFlexcanFreq(0, kClockFlexcanSrcBusClk);
    }
    else
    {
        canPeClk = CLOCK_SYS_GetFlexcanFreq(0, kClockFlexcanSrcOsc0erClk);
    }

    switch (canPeClk)
    {
        case 60000000:
            result = FLEXCAN_DRV_SetBitrate(instance, &bitRateTable60Mhz[0]); // 125kbps
            break;
        case 48000000:
            //result = FLEXCAN_DRV_SetBitrate(instance, &bitRateTable48Mhz[0]); // 125kbps
            break;
        default:
            if ((canPeClk > 74990000) && (canPeClk <= 75000000))
            {
            result = FLEXCAN_DRV_SetBitrate(instance, &bitRateTable75Mhz[0]); // 125kbps
            }
            else
            {
              PRINTF("\r\nFLEXCAN bitrate table not available for PE clock: %d", canPeClk);
              return kStatus_FLEXCAN_Fail;
            }
    }

    if (result)
    {
        numErrors++;
        PRINTF("\r\nFLEXCAN set bitrate failed. result: 0x%lx", result);
    }

    FLEXCAN_DRV_SetRxMaskType(instance, kFlexCanRxMaskIndividual);

    FLEXCAN_DRV_SetRxMaskType(instance, kFlexCanRxMaskGlobal);
    result = FLEXCAN_DRV_SetRxMbGlobalMask(instance, kFlexCanMsgIdStd, 0x123);
    if (result)
    {
        numErrors++;
        PRINTF("\r\nFLEXCAN set rx MB global mask. result: 0x%lx", result);
    }

    // Standard ID
    result = FLEXCAN_DRV_SetRxIndividualMask(instance, kFlexCanMsgIdStd, rxMailboxNum, 0x7FF);
    if(result)
    {
        numErrors++;
        PRINTF("\r\nFLEXCAN set rx individual mask with standard ID fail. result: 0x%1x", result);
    }

    // Extern ID
    result = FLEXCAN_DRV_SetRxIndividualMask(instance, kFlexCanMsgIdExt, rxMailboxNum, 0x123);
    if(result)
    {
        numErrors++;
        PRINTF("\r\nFLEXCAN set rx individual mask with standard ID fail. result: 0x%1x", result);
    }
    while(1)
    {
        // Transfer data and receive through loopback interface
        send_data();

        if (numErrors != 0)
        {
            return kStatus_FLEXCAN_Fail;
        }
        // Wait for press keyboard
        PRINTF("\r\nPress any key to run again!");
        GETCHAR();
    }

    while(1)
    {
        // Main routine that simply echoes received characters forever

        // First, get character
        receiveBuff = GETCHAR();

        // Now echo the received character
        PUTCHAR(receiveBuff);
    }
}
