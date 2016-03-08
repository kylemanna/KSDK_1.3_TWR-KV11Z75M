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

#include "board.h"
#include "fsl_uart_driver.h"
#include "fsl_clock_manager.h"

///////////////////////////////////////////////////////////////////////////////
//  Const
///////////////////////////////////////////////////////////////////////////////
const uint8_t buffStart[]   = "\r\n++++++++++++++++ UART Send/Receive Blocking Example Start +++++++++++++++++\r\n";
const uint8_t bufferData1[] = "\r\nType characters from keyboard, the board will receive and then echo them to terminal screen\r\n";

///////////////////////////////////////////////////////////////////////////////
//  Codes
///////////////////////////////////////////////////////////////////////////////
/*
 * This example will show transmit/receive UART's driver, the efficiency of the
 * transmit/receive drivers with using blocking method. Transfer data between
 * board and PC. Board will transfer and receive characters with PC through
 * UART interface. Type characters from keyboard, the board will receive and
 * then echo them to terminal screen. Look for intructions output to the terminal.
 */

/*!
 * @brief Check send/receive blocking functionality
 *
 */
int main(void)
{
    uint8_t rxChar = 0;
    uint32_t byteCountBuff = 0;

    uart_state_t uartState;

    uart_user_config_t uartConfig = {
        .bitCountPerChar = kUart8BitsPerChar,
        .parityMode      = kUartParityDisabled,
        .stopBitCount    = kUartOneStopBit,
        .baudRate        = BOARD_DEBUG_UART_BAUD
    };

    // Enable clock for PORTs, setup board clock source, config pin
    hardware_init();

    // Call OSA_Init to setup LP Timer for timeout
    OSA_Init();

    // Initialize the uart module with base address and config structure
    UART_DRV_Init(BOARD_DEBUG_UART_INSTANCE, &uartState, &uartConfig);

    // Inform to start blocking example
    byteCountBuff = sizeof(buffStart);
    UART_DRV_SendDataBlocking(BOARD_DEBUG_UART_INSTANCE, buffStart, byteCountBuff, 1000u);

    // Inform user of what to do
    byteCountBuff = sizeof(bufferData1);
    UART_DRV_SendDataBlocking(BOARD_DEBUG_UART_INSTANCE, bufferData1, byteCountBuff, 1000u);

    // Send/receive blocking function
    while(true)
    {
        // Wait to receive input data
        if (kStatus_UART_Success == UART_DRV_ReceiveDataBlocking(BOARD_DEBUG_UART_INSTANCE, &rxChar, 1u, OSA_WAIT_FOREVER))
        {
            // Echo received character
            UART_DRV_SendDataBlocking(BOARD_DEBUG_UART_INSTANCE, &rxChar, 1u, 1000u);
        }
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
