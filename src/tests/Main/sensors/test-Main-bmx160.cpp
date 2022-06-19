/* Copyright (c) 2020 Skyward Experimental Rocketry
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
#include <drivers/timer/GeneralPurposeTimer.h>
#include <sensors/BMX160/BMX160.h>
#include <utils/Debug.h>

#include "kernel/logging.h"

using namespace miosix;
using namespace Boardcore;

SPIBus bus(SPI4);

BMX160* sensor = nullptr;
uint32_t tick  = 0;

void __attribute__((used)) EXTI3_IRQHandlerImpl()
{
    tick = TimestampTimer::getTimestamp();

    if (sensor)
        sensor->IRQupdateTimestamp(tick);
}

int main()
{
    miosix::sensors::bmx160::cs::high();

    enableExternalInterrupt(miosix::sensors::bmx160::intr::getPin().getPort(),
                            miosix::sensors::bmx160::intr::getPin().getNumber(),
                            InterruptTrigger::FALLING_EDGE);

    BMX160Config config;
    config.fifoMode              = BMX160Config::FifoMode::HEADER;
    config.accelerometerDataRate = BMX160Config::OutputDataRate::HZ_25;
    config.gyroscopeDataRate     = BMX160Config::OutputDataRate::HZ_25;
    config.magnetometerRate      = BMX160Config::OutputDataRate::HZ_25;
    config.fifoInterrupt         = BMX160Config::FifoInterruptPin::PIN_INT1;
    config.fifoWatermark         = 5;
    config.temperatureDivider    = 1;

    sensor = new BMX160(bus, miosix::sensors::bmx160::cs::getPin(), config);

    TRACE("Initializing BMX160...\n");

    if (!sensor->init())
    {
        TRACE("Init failed! (code: %d)\n", sensor->getLastError());
        while (1)
        {
        }
        return -1;
    }

    TRACE("Initialization complete...\n");

    // if (!sensor->selfTest())
    // {
    //     TRACE("Self-test failed! (code: %d)\n", sensor->getLastError());
    //     return -1;
    // }

    // TRACE("Self-test successful!\n");

    while (true)
    {
        printf("----------------------------\n");

        miosix::delayMs(250);

        sensor->sample();
        if (sensor->getLastError() != SensorErrors::NO_ERRORS)
        {
            TRACE("Failed to read data!\n");
            continue;
        }

        uint64_t now = TimestampTimer::getTimestamp();

        printf("Tick: %.4f s, Now: %.4f s\n", tick / 1000000.0f,
               now / 1000000.0f);
        printf("Temp: %.2f deg\n", sensor->getTemperature().temperature);
        printf("Fill: %d\n", sensor->getLastFifoSize());

        printf("----------------------------\n");
        uint8_t len = std::min(sensor->getLastFifoSize(), (uint8_t)5);

        for (uint8_t i = 0; i < len; i++)
        {
            BMX160Data data = sensor->getFifoElement(i);
            printf("Mag [%.4f s]:\t%.2f\t%.2f\t%.2f\n",
                   data.magneticFieldTimestamp / 1000000.0f,
                   data.magneticFieldX, data.magneticFieldY,
                   data.magneticFieldZ);

            printf("Gyr [%.4f s]:\t%.2f\t%.2f\t%.2f\n",
                   data.angularVelocityTimestamp / 1000000.0f,
                   data.angularVelocityX, data.angularVelocityY,
                   data.angularVelocityZ);

            printf("Acc [%.4f s]:\t%.2f\t%.2f\t%.2f\n",
                   data.accelerationTimestamp / 1000000.0f, data.accelerationX,
                   data.accelerationY, data.accelerationZ);
        }
    }

    return 0;
}
