/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <drivers/spi/SPIDriver.h>
#include <miosix.h>
#include <radio/SX1278/SX1278Defs.h>

using namespace miosix;
using namespace Boardcore;
using namespace SX1278Defs;

void setChannel(SPITransaction &spi, uint32_t freq)
{
    freq = (double)freq / 61.03515625;
    spi.writeRegister(REG_FRF_MSB, freq >> 16);
    spi.writeRegister(REG_FRF_MID, freq >> 8);
    spi.writeRegister(REG_FRF_LSB, freq);
}

void setFrequencyDeviation(SPITransaction &spi, uint32_t freqDev)
{
    freqDev = (double)freqDev / 61.03515625;
    spi.writeRegister(REG_FDEV_MSB, (freqDev >> 8) & 0x3F);
    spi.writeRegister(REG_FDEV_LSB, freqDev);
}

void setBitRate(SPITransaction &spi, uint32_t bitRate)
{
    bitRate = 32e6 / bitRate;
    spi.writeRegister(REG_BITRATE_MSB, bitRate >> 8);
    spi.writeRegister(REG_BITRATE_LSB, bitRate);
}

void setPreambleLength(SPITransaction &spi, uint16_t length)
{
    spi.writeRegister(REG_PREAMBLE_MSB, length >> 8);
    spi.writeRegister(REG_PREAMBLE_LSB, length);
}

int main()
{
    GpioPin sck(GPIOE_BASE, 2);
    GpioPin miso(GPIOE_BASE, 5);
    GpioPin mosi(GPIOE_BASE, 6);
    GpioPin cs(GPIOC_BASE, 1);

    sck.mode(Mode::ALTERNATE);
    sck.alternateFunction(5);
    miso.mode(Mode::ALTERNATE);
    miso.alternateFunction(5);
    mosi.mode(Mode::ALTERNATE);
    mosi.alternateFunction(5);
    cs.mode(Mode::OUTPUT);
    cs.high();

    SPIBus bus(SPI4);
    SPIBusConfig config;

    SPISlave slave(bus, cs, config);

    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);

    uint32_t freq    = 434000000;
    uint32_t freqDev = 50000;
    uint32_t bitRate = 10000;

    // Check version value
    {
        auto value = spi.readRegister(REG_VERSION);

        printf("Device version: 0x%02X\n", value);
    }

    // Set working center frequency
    setChannel(spi, freq);

    // Rx chain calibration
    {
        // Cut the power amplifier
        spi.writeRegister(REG_PA_CONFIG, 0x00);

        // Start Rx chain calibration
        spi.writeRegister(REG_IMAGE_CAL, 0x40);  // ImageCalStart set to 1

        Thread::sleep(10);  // The calibration takes approximately 10 ms

        // Wait for image calibration completion
        while ((spi.readRegister(REG_IMAGE_CAL) & 0x20) == 0x20)
            Thread::sleep(1);

        printf("Rx chain calibration done\n");
    }

    // By default the modem is already configured for FSK
    // It is also in standby

    // Set Tx and Rx configuration
    {
        // On RA-01 only the PA_BOOST pin is connected
        // Set the amplifier to 17dBm
        // - 0x80: Selected PA_BOOST
        // - 0x0F: Pout set to 17dBm
        spi.writeRegister(REG_PA_CONFIG, 0x8F);

        // Don't set Pout to 20dBm, REG_PA_DAC is not changed
        // They can be sustained only at 1% transmission duty cycle

        setFrequencyDeviation(spi, freqDev);
        setBitRate(spi, bitRate);

        setPreambleLength(spi, 3);

        // Set packet configuration
        // - 0x80: Variable length
        // - 0x00: DC free encoding
        // - 0x00: CRC calculation off
        // - 0x00: Address filtering off
        // - 0x00: CCITT CRC implementation
        spi.writeRegister(REG_PACKET_CONFIG_1, 0x80);

        // Set receiver filter bandwidth
        spi.writeRegister(REG_RX_BW, 0x02);
        spi.writeRegister(REG_AFC_BW, 0x02);

        // Set payload length
        // spi.writeRegister(REG_PACKET_PAYLOAD_LENGTH, 8);

        printf("Rx and Tx configuration done\n");
    }

    // Send a packet every second
    while (true)
    {
        // Go into transmitter mode
        spi.writeRegister(REG_OP_MODE, 0x08 | 0x03);

        delayMs(5 * 8 * 2);

        // Write data into the fifo
        uint8_t data[] = {0x00, 0xFF, 0x0F, 0x0F, 0x12, 0x34, 0x56, 0x78};
        spi.writeRegister(REG_FIFO, static_cast<uint8_t>(8));
        spi.writeRegisters(REG_FIFO, data, 8);

        // Wait for packet transmission
        delayUs(1e6 / bitRate * (3 + 4 + 8) * 8);

        // printf("Current operation mode: 0x%02X\n",
        //        spi.readRegister(REG_OP_MODE));
        // printf("REG_IRQ_FLAG_1 0x%02X\n", spi.readRegister(REG_IRQ_FLAGS_1));
        // printf("REG_IRQ_FLAG_2 0x%02X\n", spi.readRegister(REG_IRQ_FLAGS_2));

        // Wait for the transmitter to be ready
        // while (spi.readRegister(REG_IRQ_FLAGS_1) == 0x20)
        //     Thread::sleep(1);

        // printf("Now in Tx mode\n");

        // Wait for packet sent
        // while (true)
        // {
        //     uint8_t flags = spi.readRegister(REG_IRQ_FLAGS_2);

        //     printf("REG_IRQ_FLAG_2 %02X\n", flags);
        //     if (flags == 0x8)
        //     {
        //         break;
        //     }

        //     Thread::sleep(1);
        // }

        // Go into standby mode
        spi.writeRegister(REG_OP_MODE, 0x08 | 0x01);

        // printf("Now in sleep mode\n");

        // printf("Current operation mode: 0x%02X\n",
        //        spi.readRegister(REG_OP_MODE));

        // printf("[%.2f] Packet sent\n", getTick() / 1e3);
        // printf("%d\n", count);
        delayMs(1);
    }

    while (true)
        Thread::sleep(1000);
}

/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Davide Mor
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <drivers/interrupt/external_interrupts.h>
#include <drivers/usart/USART.h>
#include <filesystem/console/console_device.h>
#include <radio/SX1278/SX1278.h>

#include <thread>

using namespace miosix;
using namespace Boardcore;

SX1278 *sx1278 = nullptr;
USART *usart   = nullptr;

void __attribute__((used)) EXTI10_IRQHandlerImpl()
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

void recvLoop();

void sendLoop();

int main()
{
    GpioPin sck(GPIOE_BASE, 2);
    GpioPin miso(GPIOE_BASE, 5);
    GpioPin mosi(GPIOE_BASE, 6);
    GpioPin cs(GPIOC_BASE, 1);
    GpioPin dio(GPIOF_BASE, 10);

    sck.mode(Mode::ALTERNATE);
    sck.alternateFunction(5);
    miso.mode(Mode::ALTERNATE);
    miso.alternateFunction(5);
    mosi.mode(Mode::ALTERNATE);
    mosi.alternateFunction(5);
    cs.mode(Mode::OUTPUT);
    cs.high();

    usart = new USART(USART1, USARTInterface::Baudrate::B19200);

    enableExternalInterrupt(dio.getPort(), dio.getNumber(),
                            InterruptTrigger::RISING_EDGE);

    // Run default configuration
    SX1278::Config config;

    SX1278::Error err;

    SPIBus bus(SPI4);
    sx1278 = new SX1278(bus, cs);

    printf("[sx1278] Configuring sx1278...\n");
    if ((err = sx1278->init(config)) != SX1278::Error::NONE)
    {
        while (true)
        {
            printf("[SX1278] Initialization failed\n");
            Thread::sleep(1000);
        }
    }
    printf("[sx1278] Initialization complete!\n");

    usart->init();

    std::thread recv([]() { recvLoop(); });
    std::thread send([]() { sendLoop(); });

    while (true)
        miosix::Thread::sleep(100);

    return 0;
}

void recvLoop()
{
    uint8_t msg[256];

    while (true)
    {
        int len = sx1278->receive(msg, sizeof(msg));
        if (len > 0)
            usart->write(msg, len);
    }
}

void sendLoop()
{
    uint8_t msg[63];

    while (true)
    {
        // int len = usart->read(msg, sizeof(msg));
        // if (len > 0)
        // {
        //     ledOn();
        //     sx1278->send(msg, len);
        //     ledOff();
        // }
        sx1278->send(msg, 63);
        // Thread::sleep(50);
    }
}