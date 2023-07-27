/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include <Motor/BoardScheduler.h>
#include <Motor/Buses.h>
#include <Motor/Sensors/Sensors.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace Motor;

int main()
{
    printf("APB2: %ld\n",
           ClockUtils::getAPBPeripheralsClock(ClockUtils::APB::APB2));

    auto scheduler = new BoardScheduler();

    auto sensors =
        new Sensors(scheduler->getScheduler(miosix::PRIORITY_MAX - 1));

    ModuleManager& moduleManager = ModuleManager::getInstance();
    moduleManager.insert<BoardScheduler>(scheduler);
    moduleManager.insert<Sensors>(sensors);
    moduleManager.insert<Buses>(new Buses());

    sensors->start();
    sensors->calibrate();

    printf("Sensors started\n");

    while (true)
    {
        printf("Average CPU usage: %.1f%%\n", CpuMeter::getCpuStats().mean);

        if (sensors->adc != nullptr && sensors->battery != nullptr)
        {
            auto adcData     = sensors->adc->getLastSample();
            auto batteryData = sensors->battery->getLastSample();
            printf("[%.2fs]\tADC:\t%f %f %f\n", adcData.timestamp / 1e6,
                   adcData.temperature, adcData.vBat, batteryData.batVoltage);
        }

        // WARNING: Fails self test
        if (sensors->lsm6 != nullptr)
        {
            auto lsm6Data = sensors->lsm6->getLastSample();
            printf("[%.2fs]\tLSM6:\t%fm/s^2 %fm/s^2 %fm/s^2\n",
                   lsm6Data.accelerationTimestamp / 1e6,
                   lsm6Data.accelerationX * 0.001,
                   lsm6Data.accelerationY * 0.001,
                   lsm6Data.accelerationZ * 0.001);
            printf("[%.2fs]\tLSM6:\t%frad/s %frad/s %frad/s\n",
                   lsm6Data.angularSpeedTimestamp / 1e6,
                   lsm6Data.angularSpeedX * 0.001,
                   lsm6Data.angularSpeedY * 0.001,
                   lsm6Data.angularSpeedZ * 0.001);
        }

        // WARNING: The values are wrong
        if (sensors->h3lis != nullptr)
        {
            auto h3lisData = sensors->h3lis->getLastSample();
            printf("[%.2fs]\tH3LIS:\t%fm/s %fm/s %fm/s\n",
                   h3lisData.accelerationTimestamp / 1e6,
                   h3lisData.accelerationX, h3lisData.accelerationY,
                   h3lisData.accelerationZ);
        }

        if (sensors->lis2 != nullptr)
        {
            auto lis2Data = sensors->lis2->getLastSample();
            printf("[%.2fs]\tLIS2:\t%f %f %f\n",
                   lis2Data.magneticFieldTimestamp / 1e6,
                   lis2Data.magneticFieldX, lis2Data.magneticFieldY,
                   lis2Data.magneticFieldZ);
        }

        if (sensors->lps22 != nullptr)
        {
            auto lps22Data = sensors->lps22->getLastSample();
            printf("[%.2fs]\tLPS22:\t%f %f\n",
                   lps22Data.pressureTimestamp / 1e6, lps22Data.pressure,
                   lps22Data.temperature);
        }

        if (sensors->max != nullptr)
        {
            auto maxData = sensors->max->getLastSample();
            printf("[%.2fs]\tMAX:\t%f° %f°\n",
                   maxData.temperatureTimestamp / 1e6, maxData.temperature,
                   maxData.coldJunctionTemperature);
        }

        if (sensors->chamberPressure != nullptr)
        {
            auto chamberData = sensors->chamberPressure->getLastSample();
            printf("[%.2fs]\tCHAMBER:\t%fbar\n",
                   chamberData.pressureTimestamp / 1e6, chamberData.pressure);
        }

        if (sensors->tankPressure1 != nullptr)
        {
            auto chamberData = sensors->tankPressure1->getLastSample();
            printf("[%.2fs]\tTANK1:\t\t%fbar\n",
                   chamberData.pressureTimestamp / 1e6, chamberData.pressure);
        }

        if (sensors->tankPressure2 != nullptr)
        {
            auto chamberData = sensors->tankPressure2->getLastSample();
            printf("[%.2fs]\tTANK2:\t\t%fbar\n",
                   chamberData.pressureTimestamp / 1e6, chamberData.pressure);
        }

        if (sensors->servosCurrent != nullptr)
        {
            auto servosCurrentData = sensors->servosCurrent->getLastSample();
            printf("[%.2fs]\tSERVO:\t\t%fA\n",
                   servosCurrentData.voltageTimestamp / 1e6,
                   servosCurrentData.current);
        }

        Thread::sleep(1000);
    }
}
