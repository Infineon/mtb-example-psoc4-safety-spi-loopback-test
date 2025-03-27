/******************************************************************************
* File Name:   main.c
* 
* Description:
* This file provides example usage of SCB-SPI self tests for PSoC 4.
*
* Related Document:See README.md
*
********************************************************************************
* Copyright 2023-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
********************************************************************************/

/*******************************************************************************
* Includes
********************************************************************************/

#include <stdio.h>
#include "cy_pdl.h"
#include "cybsp.h"

#include "SelfTest_SPI_SCB.h"
#include "cycfg_pins.h"

/*******************************************************************************
* Macros
********************************************************************************/

#define MAX_INDEX_VAL 0xFFF0u
#define PRINT_DELAY_MS 10u
#define MUX_SWITCHING_DELAY_US 100u

/*******************************************************************************
* Global Variables
*******************************************************************************/

static cy_stc_scb_spi_context_t CYBSP_DUT_SPI_context;
char uart_print_buff[100]={0};

/*******************************************************************************
* Function Prototypes
********************************************************************************/

static void CYBSP_DUT_SPI_Interrupt(void);
static void SelfTest_SPI_SCB_Init(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* The main function performs the following tasks:
*    1. Initializes the device, board peripherals, and retarget-io for prints.
*    2. Initializes the Smart-IO based on the design.modus file.
*    3. Calls the SelfTest_SPI_SCB_Init function to initialize the SPI component.
*    4. Bypasses all SmartIO configurations to enable the normal operation of
*       the SPI block.
*    5. Calls abort APIs from the SPI PDL to prevent interrupt-driven transfers
*       from triggering during the self-test. 
*    6. Enables loopback by disabling the bypass on all SmartIO configurations
*       to test the SPI block.
*    7. Calls the SelfTest_SPI_SCB API to test the SPI SCB IP.
*    8. Disables loopback and bypasses SmartIO configurations to restore normal
*       operation of the SPI block.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/

int main(void)
{
    cy_rslt_t result;
    cy_en_smartio_status_t smart_res;
    cy_stc_scb_uart_context_t CYBSP_UART_context;

    uint8_t ret;

    uint16_t count = 0u;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();
    
    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\x1b[2J\x1b[;H");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\nClass-B Safety Test: SPI Loopback\r\n");

    /* Configure the Smart I/O block 2 */
    if(CY_SMARTIO_SUCCESS != Cy_SmartIO_Init(CYBSP_SMARTIO_SPI_LOOPBACK_HW, &CYBSP_SMARTIO_SPI_LOOPBACK_config))
    {
        /* Insert error handling */
        CY_ASSERT(0x0u);
    }

    /* Enable Smart I/O block*/
    Cy_SmartIO_Enable(CYBSP_SMARTIO_SPI_LOOPBACK_HW);

    /* Init SPI SelfTest*/
    SelfTest_SPI_SCB_Init();

    /* Bypass all the SmartIO configuration for normal mode */
    Cy_SmartIO_Disable(CYBSP_SMARTIO_SPI_LOOPBACK_HW);
    smart_res = Cy_SmartIO_SetChBypass(CYBSP_SMARTIO_SPI_LOOPBACK_HW, CY_SMARTIO_CHANNEL_ALL);
    if(smart_res != CY_SMARTIO_SUCCESS)
    {
        /* Insert error handling */
        CY_ASSERT(0x0u);
    }       
    Cy_SmartIO_Enable(CYBSP_SMARTIO_SPI_LOOPBACK_HW);

    /* Need to clear buffer after MUX switch */
    Cy_SysLib_DelayUs(MUX_SWITCHING_DELAY_US);
    Cy_SCB_SPI_ClearRxFifo(CYBSP_DUT_SPI_HW);
    Cy_SCB_SPI_ClearTxFifo(CYBSP_DUT_SPI_HW);

    for (;;)
    {
        /* If using high level SPI APIs, run the Cy_SCB_SPI_AbortTransfer
         * before the self test to disable the TX and RX interrupt sources,
         * clear the TX and RX FIFOs, and the reset the context status*/
        Cy_SCB_SPI_AbortTransfer(CYBSP_DUT_SPI_HW, &CYBSP_DUT_SPI_context);

        /* Turn on loopback by disabling the bypass on all the SmartIO configuration for test mode */
        Cy_SmartIO_Disable(CYBSP_SMARTIO_SPI_LOOPBACK_HW);
        smart_res = Cy_SmartIO_SetChBypass(CYBSP_SMARTIO_SPI_LOOPBACK_HW,CY_SMARTIO_CHANNEL_NONE);
        if(smart_res != CY_SMARTIO_SUCCESS)
        {
            /* Insert error handling */
            CY_ASSERT(0x0u);
        }               
        Cy_SmartIO_Enable(CYBSP_SMARTIO_SPI_LOOPBACK_HW);

        /* Clear RX, TX buffers */
        Cy_SCB_SPI_ClearRxFifo(CYBSP_DUT_SPI_HW);
        Cy_SCB_SPI_ClearTxFifo(CYBSP_DUT_SPI_HW);

        /*******************************/
        /* Run SPI Self Test...        */
        /*******************************/
        ret = SelfTest_SPI_SCB(CYBSP_DUT_SPI_HW);

        /* Turn off loopback by enabling the bypass on all the SmartIO configuration for normal mode */
        Cy_SmartIO_Disable(CYBSP_SMARTIO_SPI_LOOPBACK_HW);
        smart_res = Cy_SmartIO_SetChBypass(CYBSP_SMARTIO_SPI_LOOPBACK_HW, CY_SMARTIO_CHANNEL_ALL);
        if(smart_res != CY_SMARTIO_SUCCESS)
        {
            /* Insert error handling */
            CY_ASSERT(0x0u);
        }                       
        Cy_SmartIO_Enable(CYBSP_SMARTIO_SPI_LOOPBACK_HW);

        /* Need to clear buffer after MUX switch */
        Cy_SysLib_DelayUs(MUX_SWITCHING_DELAY_US);
        Cy_SCB_SPI_ClearRxFifo(CYBSP_DUT_SPI_HW);
        Cy_SCB_SPI_ClearTxFifo(CYBSP_DUT_SPI_HW);

        if ((PASS_COMPLETE_STATUS != ret) && (PASS_STILL_TESTING_STATUS != ret))
        {
            /* Process error */
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\nSPI SCB test: error \r\n");
            while (1);
        }

        /* Print test counter */
        sprintf(uart_print_buff, "\rSPI SCB loopback testing... count=%d", count);
        Cy_SCB_UART_PutString(CYBSP_UART_HW, uart_print_buff);

        Cy_SysLib_Delay(PRINT_DELAY_MS);
        
        count++;
        if (count > MAX_INDEX_VAL)
        {
            count = 0u;
        }

    }

}

/*****************************************************************************
* Function Name: SelfTest_SPI_SCB_Init
******************************************************************************
*
* Summary:
*  Init SPI component and clear internal SPI buffers
*  Should be called once before tests
*
* Parameters:
*  NONE
*
* Return:
*  NONE
*
*****************************************************************************/
static void SelfTest_SPI_SCB_Init(void)
{

    cy_en_scb_spi_status_t result;
    cy_en_sysint_status_t status;

    /* Start SPI component */
    result = Cy_SCB_SPI_Init(CYBSP_DUT_SPI_HW, &CYBSP_DUT_SPI_config, &CYBSP_DUT_SPI_context);
    if( result != CY_SCB_SPI_SUCCESS)
    {
        CY_ASSERT(0u);
    }

    /* Set active slave select to line 0 */
    Cy_SCB_SPI_SetActiveSlaveSelect(CYBSP_DUT_SPI_HW, CY_SCB_SPI_SLAVE_SELECT0);

    /* Populate configuration structure */

    const cy_stc_sysint_t CYBSP_DUT_SPI_SCB_IRQ_cfg =
    {
        .intrSrc      = CYBSP_DUT_SPI_IRQ,
        .intrPriority = 3u
    };

    /* Hook interrupt service routine and enable interrupt */
    status = Cy_SysInt_Init(&CYBSP_DUT_SPI_SCB_IRQ_cfg, &CYBSP_DUT_SPI_Interrupt);
    if(status != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0u);
    }
    /* Enable interrupt in NVIC */
    NVIC_EnableIRQ(CYBSP_DUT_SPI_IRQ);


    /* Enable the SPI Master block */
    Cy_SCB_SPI_Enable(CYBSP_DUT_SPI_HW);

    /* Clear RX, TX buffers */
    Cy_SCB_SPI_ClearRxFifo(CYBSP_DUT_SPI_HW);
    Cy_SCB_SPI_ClearTxFifo(CYBSP_DUT_SPI_HW);

}

/*******************************************************************************
 * Function Name: CYBSP_DUT_SPI_Interrupt
 *******************************************************************************
 *
 * Summary: Invokes the Cy_SCB_SPI_Interrupt() PDL driver function.
 *
 * Parameters:
 *  NONE
 *
 * Return:
 *  NONE
 ******************************************************************************/
static void CYBSP_DUT_SPI_Interrupt(void)
{
    Cy_SCB_SPI_Interrupt(CYBSP_DUT_SPI_HW, &CYBSP_DUT_SPI_context);
}

/* [] END OF FILE */
