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
#include <filesystem/console/console_device.h>
#include <radio/SX1278/SX1278.h>
#include <interfaces-impl/hwmapping.h>

#include <thread>

using namespace miosix;
using namespace Boardcore;

const char *stringFromErr(SX1278::Error err)
{
    switch (err)
    {
        case SX1278::Error::BAD_VALUE:
            return "Error::BAD_VALUE";

        case SX1278::Error::BAD_VERSION:
            return "Error::BAD_VERSION";

        default:
            return "<unknown>";
    }
}

SX1278 *sx1278 = nullptr;

void __attribute__((used)) EXTI6_IRQHandlerImpl()
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

void recvLoop()
{
    uint8_t msg[256];
    while (1)
    {
        int len = sx1278->receive(msg, sizeof(msg));
        if (len > 0)
        {
            auto serial = miosix::DefaultConsole::instance().get();
            serial->writeBlock(msg, len, 0);
        }
    }
}

void sendLoop()
{
    uint8_t msg[63];
    while (1)
    {
        auto serial = miosix::DefaultConsole::instance().get();
        int len     = serial->readBlock(msg, sizeof(msg), 0);
        if (len > 0)
        {
            sx1278->send(msg, len);
        }
    }
}

int main()
{
    /*
    This entrypoint uses the RA01 module instead of the not-so-working sx127x
    */

    // Enable dio0 interrupt
    enableExternalInterrupt(GPIOF_BASE, 6, InterruptTrigger::RISING_EDGE);

    // Run default configuration
    SX1278::Config config;
    SX1278::Error err;

    SPIBus bus(SPI4);
    GpioPin cs = peripherals::ra01::cs::getPin();

    sx1278 = new SX1278(bus, cs);

    printf("\n[sx1278] Configuring sx1278...\n");
    if ((err = sx1278->init(config)) != SX1278::Error::NONE)
    {
        printf("[sx1278] sx1278->init error: %s\n", stringFromErr(err));
        return -1;
    }

    printf("\n[sx1278] Initialization complete!\n");

    std::thread recv([]() { recvLoop(); });
    std::thread send([]() { sendLoop(); });

    for (;;)
    {
        miosix::Thread::sleep(100);
    }

    return 0;
}