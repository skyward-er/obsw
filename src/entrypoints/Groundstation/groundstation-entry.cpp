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
#include <interfaces-impl/hwmapping.h>
#include <radio/SX1278/SX1278.h>

#include <thread>

#define USE_RA01

using namespace miosix;
using namespace Boardcore;

SX1278 *sx1278 = nullptr;
USART *usart   = nullptr;

#ifdef USE_RA01
void __attribute__((used)) EXTI6_IRQHandlerImpl()
#else
void __attribute__((used)) EXTI3_IRQHandlerImpl()
#endif
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

void recvLoop();

void sendLoop();

int main()
{
    usart = new USART(UART5, USARTInterface::Baudrate::B19200);
    usart->init();

    // Enable dio0 interrupt
#ifdef USE_RA01
    enableExternalInterrupt(GPIOF_BASE, 6, InterruptTrigger::RISING_EDGE);
#else
    enableExternalInterrupt(GPIOE_BASE, 3, InterruptTrigger::RISING_EDGE);
#endif

    // Run default configuration
    SX1278::Config config;
    SX1278::Error err;

    SPIBus bus(SPI4);
#ifdef USE_RA01
    GpioPin cs = peripherals::ra01::cs::getPin();
#else
    GpioPin cs = peripherals::sx127x::cs::getPin();
#endif

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

    std::thread recv([]() { recvLoop(); });
    std::thread send([]() { sendLoop(); });

    while (true)
        miosix::Thread::sleep(100);
}

void recvLoop()
{
    uint8_t msg[256];

    while (true)
    {
        int len = sx1278->receive(msg, sizeof(msg));
        if (len > 0)
        {
            usart->writeString("This is the message:\r\n");
            usart->write(msg, len);
        }
    }
}

void sendLoop()
{
    uint8_t msg[63];

    while (true)
    {
        int len = usart->read(msg, sizeof(msg));

        if (len > 0)
        {
            ledOn();
            sx1278->send(msg, len);
            ledOff();
        }
    }
}
