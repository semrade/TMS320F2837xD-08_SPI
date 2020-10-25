/***********************************************************************************
 * File              :TS_SysMng_SPI_RxTx_Int.c
 *
 * Title             :
 *
 * Author            :Tarik SEMRADE
 *
 * Created on        :Mar 19, 2020
 *
 * Version           :
 *
 * Description       :
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
 **********************************************************************************/

/**********************************************************************************
 *  Included Files
 *
 *********************************************************************************/
#include "TS_SysMng_SPI_RxTx_Int.h"
#include "driverlib.h"
#include "device.h"

/**********************************************************************************
 *  Global Variables
 *
 *********************************************************************************/
volatile uint16_t sData[16];                // Send data buffer
volatile uint16_t rData[16];                // Receive data buffer
/**********************************************************************************
 * \function:      TS_SysMng_InitSpiBMaster
 * \brief             Function to configure SPI B as master with FIFO enabled.
 * \param[in]      void
 * \return          void
 **********************************************************************************/
void TS_SysMng_InitSpiBMaster(void)
{

    /* Must put SPI into reset before configuring it */
    SPI_disableModule(SPIB_BASE);

    /* SPI configuration. Use a 500kHz SPICLK and 16-bit word size.*/
    /* Clock polarity and phase*/
    /* Master */
    /* Speed 500K */
    /* 16 bites data */
    SPI_setConfig(SPIB_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_MASTER, SPEED_SPI_MASTER, DATA_LENTGH);
    /* Disable LB*/
    SPI_disableLoopback(SPIB_BASE);

    /* Free Run to inf */
    SPI_setEmulationMode(SPIB_BASE, SPI_EMULATION_FREE_RUN);
    SPI_disableFIFO(SPIB_BASE);
    SPI_enableTalk(SPIB_BASE);

#if TX_FIFO

    /* FIFO and interrupt configuration */
    SPI_enableFIFO(SPIB_BASE);
    SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_TXFF);

    /* TX FIFO Level interrupt = 8 */
    SPI_setFIFOInterruptLevel(SPIB_BASE, SPI_FIFO_TX8, SPI_FIFO_RX2);
    SPI_enableInterrupt(SPIB_BASE, SPI_INT_TXFF);

    /* Delay between words*/
    SPI_setTxFifoTransmitDelay(SPIB_BASE,DELAY_CYCLE_WORD);

#endif


    /* Configuration complete. Enable the module. */
    SPI_enableModule(SPIB_BASE);
}
/**********************************************************************************
 * \function:      TS_SysMng_InitSpiASlave
 * \brief             Function to configure SPI A as slave with FIFO enabled.
 * \param[in]      void
 * \return          void
 **********************************************************************************/
void TS_SysMng_InitSpiASlave(void)
{

    /* Must put SPI into reset before configuring it */
    SPI_disableModule(SPIA_BASE);

    /* SPI configuration. Use a 500kHz SPICLK and 16-bit word size. */
    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_SLAVE, SPEED_SPI_MASTER, DATA_LENTGH);
    SPI_disableLoopback(SPIA_BASE);
    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_FREE_RUN);

    /* FIFO and interrupt configuration */
    SPI_enableFIFO(SPIA_BASE);
    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RXFF);

    /* RX FIFO Level interrupt = 16 */
    SPI_setFIFOInterruptLevel(SPIA_BASE, SPI_FIFO_TX2, SPI_FIFO_RX16);
    SPI_enableInterrupt(SPIA_BASE, SPI_INT_RXFF);

    /* Delay between words */
    SPI_setTxFifoTransmitDelay(SPIA_BASE,DELAY_CYCLE_WORD);

    /*  Configuration complete. Enable the module. */
    SPI_enableModule(SPIA_BASE);
}
/**********************************************************************************
 * \function:       TS_SysMng_SpibTxFIFOISR
 * \brief              PIE1.1 @0x000D40  ADC-A interrupt #1
 * \param[in]       void
 * \return           void
 **********************************************************************************/
__interrupt void TS_SysMng_SpibTxFIFOISR(void)
{
    uint16_t i;

    /* Checking the FIFO level before each write to make sure you don't overrun the TX FIFO */
    if (SPI_getTxFIFOStatus(SPIB_BASE) <=16)
    {
        /* Send data */
        for (i = 0; i < DATA_LENTGH; i++)
        {
            SPI_writeDataBlockingNonFIFO(SPIB_BASE, sData[i]);
        }
    }

    /* Increment data for next cycle */
    for (i = 0; i < DATA_LENTGH; i++)
    {
        sData[i] = sData[i] + 1;
    }


    /* Clear interrupt flag and issue ACK */
    SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);
}
/**********************************************************************************
 * \function:       TS_SysMng_SpibTxFIFOISR
 * \brief              PIE1.1 @0x000D40  ADC-A interrupt #1
 * \param[in]       void
 * \return           void
 **********************************************************************************/
__interrupt void TS_SysMng_SpiaRxFIFOISR(void)
{
    uint16_t i;

    /* Read data */
    for (i = 0; i < DATA_LENTGH; i++)
    {
        rData[i] = SPI_readDataBlockingFIFO(SPIA_BASE);
    }

    /* Check received data */
    for (i = 0; i < DATA_LENTGH; i++)
    {
        if ((1 + rData[i]) != sData[i])
        {
            /* Something went wrong. rData doesn't contain expected data. */
            Example_Fail = 1;
            ESTOP0;
        }
    }

    /* Clear interrupt flag and issue ACK */
    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);

    Example_PassCount++;
}
/**********************************************************************************
 * \function:       TS_SysMng_ConfigSpiAandBGPIOs
 * \brief              PIE1.1 @0x000D40  ADC-A interrupt #1
 * \param[in]       void
 * \return           void
 **********************************************************************************/
void TS_SysMng_ConfigSpiAandBGPIOs(void)
{

    /*
     *   -External Connections:
     *   -GPIO58 and GPIO63 - SPI-MOSI
     *   -GPIO59 and GPIO64 - SPI-MISO
     *   -GPIO61 and GPIO65 - SPI-STE
     *   -GPIO60 and GPIO66 - SPI-CLK
     */

    /* SPI-A configuration */
    /* GPIO59 is the SPI-MISO. */
    GPIO_setMasterCore(59, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_59_SPISOMIA);
    GPIO_setPadConfig(59, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(59, GPIO_QUAL_ASYNC);

    /* GPIO58 is the SPI-MOSI clock pin. */
    GPIO_setMasterCore(58, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_58_SPISIMOA);
    GPIO_setPadConfig(58, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(58, GPIO_QUAL_ASYNC);

    /* GPIO61 is the SPI-STEA. */
    GPIO_setMasterCore(61, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_61_SPISTEA);
    GPIO_setPadConfig(61, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(61, GPIO_QUAL_ASYNC);

    /* GPIO60 is the SPI-CLKA. */
    GPIO_setMasterCore(60, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_60_SPICLKA);
    GPIO_setPadConfig(60, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(60, GPIO_QUAL_ASYNC);

    /* SPI-B configuration */
    /* GPIO64 is the SPI-SOMIB. */
    GPIO_setMasterCore(64, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_64_SPISOMIB);
    GPIO_setPadConfig(64, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(64, GPIO_QUAL_ASYNC);

    /* GPIO63 is the SPISIMOB clock pin. */
    GPIO_setMasterCore(63, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_63_SPISIMOB);
    GPIO_setPadConfig(63, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(63, GPIO_QUAL_ASYNC);

    /* GPIO65 is the SPICLKB. */
    GPIO_setMasterCore(65, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_65_SPICLKB);
    GPIO_setPadConfig(65, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(65, GPIO_QUAL_ASYNC);

    /* GPIO66 is the SPISTEB. */
    GPIO_setMasterCore(66, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_66_SPISTEB);
    GPIO_setPadConfig(66, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(66, GPIO_QUAL_ASYNC);
}
