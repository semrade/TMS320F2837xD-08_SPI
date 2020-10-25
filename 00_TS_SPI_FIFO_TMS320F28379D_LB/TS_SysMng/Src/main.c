/***********************************************************************************
 * File                     :main.c
 *
 * Title                    :
 *
 * Author                   :Tarik SEMRADE
 *
 * Description              :
 *                            External Connections
 *                             -GPIO25 and GPIO17 - SPISOMI
 *                             -GPIO24 and GPIO16 - SPISIMO
 *                             -GPIO27 and GPIO19 - SPISTE
 *                             -GPIO26 and GPIO18 - SPICLK
 *
 * Version                  :
 *
 * Copyright (c) 2020 Tarik SEMRADE
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *********************************************************************************/

/**********************************************************************************
 *  Included Files
 *
 *********************************************************************************/
#include "F28x_Project.h"
#include "device.h"
#include "main.h"
#include "TS_SysMng_SPI_RxTx_Int.h"
/**********************************************************************************
 *  Defines
 *********************************************************************************/

/**********************************************************************************
 *  Global Variables
 *
 *********************************************************************************/

/**********************************************************************************
 * \function:       main
 * \brief           main `0` numbers
 * \param[in]       void
 * \return          void
 **********************************************************************************/
void main(void)
{
    uint16_t i;

    /* Set up system flash and turn peripheral clocks */
    InitSysCtrl();

    /* Init all gpio to input */
    InitGpio();

    /* Globally disable maskable CPU interrupts */
    DINT;

    /* Clear and disable all PIE interrupts */
    InitPieCtrl();

    /* Individually disable maskable CPU interrupts */
    IER = 0x0000;

    /* Clear all CPU interrupt flags */
    IFR = 0x0000;

    /* Populate the PIE interrupt vector table */
    InitPieVectTable();

    /***********************Interrupt linking functions*****************************/

    //Interrupt_register(INT_SPIB_TX, &TS_SysMng_SpibTxFIFOISR);
    Interrupt_register(INT_SPIA_RX, &TS_SysMng_SpiaRxFIFOISR);

    /************************Peripheral Initialization*****************************/

    /* Configure GPIOs for external communication between SPIA and SPIB. */
    TS_SysMng_ConfigSpiAandBGPIOs();

    /* Set up SPI B as master, initializing it for FIFO mode */
    TS_SysMng_InitSpiBMaster();

    /* Set up SPI A as slave, initializing it for FIFO mode */
    TS_SysMng_InitSpiASlave();

    /* Initialize the data buffers */
    for (i = 0; i < 16; i++)
    {
        sData[i] = 1;
        rData[i] = 0;
    }

    /* Enable interrupts required for this example */
    Interrupt_enable(INT_SPIA_RX);
    //Interrupt_enable(INT_SPIB_TX);

    /* Enable Global Interrupt (INTM) and realtime interrupt (DBGM) */
    EINT;
    ERTM;

    /* Infinite led loop */
    while (1)
    {

        /* Write data to the SPI B buffer */
        for (i = 0; i < 16; i++)
        {
            /* Send 16 characters, the max FIFO buffer */
            SPI_writeDataBlockingNonFIFO(SPIB_BASE, sData[i]);
        }

        /* Increment the data for a new frame of 16 words */
        for (i = 0; i < 16; i++)
        {
            sData[i] = sData[i] + 1;
        }

    }

}
