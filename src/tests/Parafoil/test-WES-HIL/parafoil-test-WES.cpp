/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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
#include <HIL.h>
#include <HILConfig.h>
#include <HIL_sensors/HILGps.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/WindEstimationScheme/WindEstimation.h>
#include <miosix.h>
#include <utils/Debug.h>

#include <utils/ModuleManager/ModuleManager.hpp>
#include <vector>

#include "thread"

using namespace miosix;
using namespace Boardcore;
using namespace Parafoil;
using namespace std;

/**
 * @brief HILSensor implementation for the HIL test of Parafoil algorithms
 */
class HILSensors : public Sensors
{
public:
    HILSensors(TaskScheduler* scheduler) : scheduler(scheduler) {}

    bool startModule() override
    {
        gps = new HILGps(HILConfig::N_DATA_GPS,
                         &Boardcore::ModuleManager::getInstance()
                              .get<HIL>()
                              ->simulator->getSensorData()
                              ->gps);

        SensorInfo info("GPS_HIL", 1000 / HILConfig::GPS_FREQ);
        sensorMap.emplace(make_pair(gps, info));

        // Create sensor manager with populated map and configured scheduler
        manager = new SensorManager(sensorMap, scheduler);
        return true;
    }

    UBXGPSData getUbxGpsLastSample() override
    {
        HILGpsData data;
        UBXGPSData ubxData;

        {
            miosix::PauseKernelLock kLock;
            data = gps->getLastSample();
        }

        ubxData.gpsTimestamp  = data.gpsTimestamp;
        ubxData.latitude      = data.latitude;
        ubxData.longitude     = data.longitude;
        ubxData.height        = data.height;
        ubxData.velocityNorth = data.velocityNorth;
        ubxData.velocityEast  = data.velocityEast;
        ubxData.velocityDown  = data.velocityDown;
        ubxData.speed         = data.speed;
        ubxData.track         = data.track;
        ubxData.positionDOP   = data.positionDOP;
        ubxData.satellites    = 6;
        ubxData.fix           = 1;

        return ubxData;
    }

private:
    SensorManager* manager = nullptr;
    Boardcore::SensorManager::SensorMap_t sensorMap;
    Boardcore::TaskScheduler* scheduler = nullptr;

    HILGps* gps = nullptr;
};

class WindEstimationMock : public WindEstimation
{
public:
    WindEstimationMock() : WindEstimation() {}
};

int main()
{
    GpioPin u2tx(GPIOA_BASE, 2);
    GpioPin u2rx(GPIOA_BASE, 3);
    u2tx.mode(Mode::ALTERNATE);
    u2tx.alternateFunction(7);
    u2rx.mode(Mode::ALTERNATE);
    u2rx.alternateFunction(7);

    ModuleManager& modules = ModuleManager::getInstance();
    USART usart2(USART2, 115200, 1000);

    // Initialize the modules
    TaskScheduler& scheduler = BoardScheduler::getInstance().getScheduler();
    Sensors* sensors         = new HILSensors(&scheduler);
    WindEstimation* wind_estimation = new WindEstimationMock();
    HIL* hil                        = new HIL(usart2);

    // Insert the modules
    if (!modules.insert<HIL>(hil))
    {
        TRACE("Error inserting HIL\n");
    }

    if (!modules.insert<Sensors>(sensors))
    {
        TRACE("Error inserting Sensor\n");
    }

    if (!modules.insert<WindEstimation>(wind_estimation))
    {
        TRACE("Error inserting wind estimation\n");
    }

    // start the scheduler
    if (!scheduler.start())
    {
        TRACE("Error starting the General Purpose Scheduler\n");
    }

    if (!hil->start())
    {
        TRACE("Error inserting HIL\n");
    }

    // Start the modules
    if (!ModuleManager::getInstance().get<Sensors>()->startModule())
    {
        TRACE("Error starting Sensors\n");
    }

    if (!ModuleManager::getInstance().get<WindEstimation>()->startModule())
    {
        TRACE("Error starting WindEstimation\n");
    }

    BoardScheduler::getInstance().getScheduler().start();
    bool simulation_started = false;
    hil->flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIMULATION_STARTED, [&]() { simulation_started = true; });

    while (!simulation_started)
    {
        HILConfig::ActuatorData actuatorData{1.0f, 0, 0};
        usart2.write(&actuatorData, sizeof(HILConfig::ActuatorData));
        Thread::sleep(100);
    }

    // Adding task to send periodically to the simulator the wind estimated
    BoardScheduler::getInstance().getScheduler().addTask(
        [&]()
        {
            auto wind = wind_estimation->getWindEstimationScheme();

            HILConfig::ActuatorData actuatorData{
                (wind_estimation->getStatus() ? 2.0f : 1.0f), wind[0], wind[1]};

            // Actually sending the feedback to the simulator
            modules.get<HIL>()->send(actuatorData);

            UBXGPSData gpsData = sensors->getUbxGpsLastSample();
            TRACE("gpsP: [%f, %f, %f]\n", gpsData.latitude, gpsData.longitude,
                  gpsData.height);
            TRACE("gpsV: [%f, %f, %f]\n", gpsData.velocityNorth,
                  gpsData.velocityEast, gpsData.velocityDown);
            actuatorData.print();
            printf("\n");
        },
        HILConfig::SIMULATION_PERIOD,
        Boardcore::TaskScheduler::Policy::RECOVER);

    wind_estimation->startWindEstimationSchemeCalibration();

    while (wind_estimation->getStatus())
    {
        Thread::sleep(1000);
    }
    TRACE("test ended wes result: n= %f, e= %f \n\n\n\n",
          wind_estimation->getWindEstimationScheme()(0),
          wind_estimation->getWindEstimationScheme()(1));
    while (1)
    {
        Thread::sleep(1000);
    }
    return 0;
}
