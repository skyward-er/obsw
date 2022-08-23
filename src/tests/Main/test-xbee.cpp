/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#ifndef RUN_SENDER
#define RUN_SENDER true
#endif
#ifndef RUN_RECEIVER
#define RUN_RECEIVER true
#endif

#include <drivers/interrupt/external_interrupts.h>
#include <logger/Logger.h>
#include <miosix.h>
#include <radio/Xbee/ATCommands.h>

using namespace miosix;
using namespace Boardcore;

using attn = Gpio<GPIOD_BASE, 7>;   // GPIO14
using rst  = Gpio<GPIOC_BASE, 13>;  // GPIO8

Xbee::Xbee* xbee = nullptr;

void __attribute__((used)) EXTI7_IRQHandlerImpl()
{
    if (xbee != nullptr)
        xbee->handleATTNInterrupt();
}

int main()
{
    attn::mode(Mode::INPUT);
    enableExternalInterrupt(GPIOD_BASE, 7, InterruptTrigger::FALLING_EDGE);

    SPIBus spi5(SPI5);
    SPIBusConfig config{};
    config.clockDivider = SPI::ClockDivider::DIV_16;

    xbee = new Xbee::Xbee(spi5, config, sensors::sx127x::cs::getPin(),
                          attn::getPin(), rst::getPin());
    Xbee::setDataRate(*xbee, true, 5000);

    while (true)
    {
        uint8_t data[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
        xbee->send(data, sizeof(data));

        delayMs(10);
    }
}
