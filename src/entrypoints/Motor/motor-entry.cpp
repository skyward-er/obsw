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

#include <Motor/Actuators/Actuators.h>
#include <Motor/BoardScheduler.h>
#include <Motor/Buses.h>
#include <Motor/CanHandler/CanHandler.h>
#include <Motor/Sensors/Sensors.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace Motor;

int main()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Overall status, if at some point it becomes false, there is a problem
    // somewhere
    bool initResult    = true;
    PrintLogger logger = Logging::getLogger("main");

    // Create modules
    BoardScheduler* scheduler = new BoardScheduler();
    Buses* buses              = new Buses();
    auto actuators =
        new Actuators(scheduler->getScheduler(miosix::PRIORITY_MAX));
    auto sensors =
        new Sensors(scheduler->getScheduler(miosix::PRIORITY_MAX - 1));
    auto canHandler =
        new CanHandler(scheduler->getScheduler(miosix::PRIORITY_MAX - 2));

    // Insert modules
    if (!modules.insert<BoardScheduler>(scheduler))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the board scheduler module");
    }

    if (!modules.insert<Buses>(buses))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the buses module");
    }

    if (!modules.insert<Actuators>(actuators))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the sensor module");
    }

    if (!modules.insert<Sensors>(sensors))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the sensor module");
    }

    if (!modules.insert<CanHandler>(canHandler))
    {
        initResult = false;
        LOG_ERR(logger, "Error inserting the CanHandler module");
    }

    // Start modules
    if (!Logger::getInstance().start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the logger module");
    }

    if (!modules.get<BoardScheduler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the board scheduler module");
    }

    if (!modules.get<Actuators>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the sensors module");
    }

    if (!modules.get<Sensors>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the sensors module");
    }

    if (!modules.get<CanHandler>()->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the CanHandler module");
    }

    // Calibration
    modules.get<Sensors>()->calibrate();

    // Set the init status inside the CAN handler
    modules.get<CanHandler>()->setInitStatus(initResult);

    // Check the init result and launch an event
    if (initResult)
    {
        miosix::led1On();
    }
    else
    {
        miosix::led2On();
    }

    while (true)
    {
        printf("Average CPU usage: %.1f%%\n", CpuMeter::getCpuStats().mean);

        auto adcData     = sensors->getADCData();
        auto batteryData = sensors->getBatteryData();
        printf("[%.2fs]\tADC:\t%f %f %f\n", adcData.timestamp / 1e6,
               adcData.temperature, adcData.vBat, batteryData.batVoltage);

        // // WARNING: Fails self test
        // auto lsm6Data = sensors->getLSM6DSRXData();
        // printf("[%.2fs]\tLSM6:\t%fm/s^2 %fm/s^2 %fm/s^2\n",
        //        lsm6Data.accelerationTimestamp / 1e6,
        //        lsm6Data.accelerationX * 0.001, lsm6Data.accelerationY *
        //        0.001, lsm6Data.accelerationZ * 0.001);
        // printf("[%.2fs]\tLSM6:\t%frad/s %frad/s %frad/s\n",
        //        lsm6Data.angularSpeedTimestamp / 1e6,
        //        lsm6Data.angularSpeedX * 0.001, lsm6Data.angularSpeedY *
        //        0.001, lsm6Data.angularSpeedZ * 0.001);

        // // WARNING: The values are wrong
        // auto h3lisData = sensors->getH3LIS331DLData();
        // printf("[%.2fs]\tH3LIS:\t%fm/s %fm/s %fm/s\n",
        //        h3lisData.accelerationTimestamp / 1e6,
        //        h3lisData.accelerationX, h3lisData.accelerationY,
        //        h3lisData.accelerationZ);

        // auto lis2Data = sensors->getLIS2MDLData();
        // printf("[%.2fs]\tLIS2:\t%f %f %f\n",
        //        lis2Data.magneticFieldTimestamp / 1e6,
        //        lis2Data.magneticFieldX, lis2Data.magneticFieldY,
        //        lis2Data.magneticFieldZ);

        // auto lps22Data = sensors->getLPS22DFData();
        // printf("[%.2fs]\tLPS22:\t%f %f\n", lps22Data.pressureTimestamp / 1e6,
        //        lps22Data.pressure, lps22Data.temperature);

        // auto maxData = sensors->getMAX31856Data();
        // printf("[%.2fs]\tMAX:\t%f° %f°\n", maxData.temperatureTimestamp /
        // 1e6,
        //        maxData.temperature, maxData.coldJunctionTemperature);

        auto adsData = sensors->getADS131M08Data();
        printf("[%.2fs]\tADS131:\t%f %f %f %f\n", adsData.timestamp / 1e6,
               adsData.getVoltage(ADS131M08Defs::Channel::CHANNEL_5).voltage,
               adsData.getVoltage(ADS131M08Defs::Channel::CHANNEL_6).voltage,
               adsData.getVoltage(ADS131M08Defs::Channel::CHANNEL_7).voltage,
               adsData.getVoltage(ADS131M08Defs::Channel::CHANNEL_1).voltage);

        auto chamberData = sensors->getChamberPressureSensorData();
        printf("[%.2fs]\tCHAMBER:\t%fbar\n",
               chamberData.pressureTimestamp / 1e6, chamberData.pressure);

        auto tank1Data = sensors->getTankPressureSensor1Data();
        printf("[%.2fs]\tTANK1:\t\t%fbar\n", tank1Data.pressureTimestamp / 1e6,
               tank1Data.pressure);

        auto tank2Data = sensors->getTankPressureSensor2Data();
        printf("[%.2fs]\tTANK2:\t\t%fbar\n", tank2Data.pressureTimestamp / 1e6,
               tank2Data.pressure);

        auto servoCurrent = sensors->getServoCurrentData();
        printf("[%.2fs]\tSERVO:\t\t%fA\n", servoCurrent.currentTimestamp / 1e6,
               servoCurrent.current);

        auto batteryVoltage = sensors->getBatteryData();
        printf("[%.2fs]\tBATTERY:\t%fV\n",
               batteryVoltage.voltageTimestamp / 1e6,
               batteryVoltage.batVoltage);

        Thread::sleep(1000);
    }
}
