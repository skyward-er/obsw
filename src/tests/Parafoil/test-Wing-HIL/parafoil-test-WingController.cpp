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
#include <HIL_sensors/HILSensor.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <Parafoil/StateMachines/WingController/WingController.h>
#include <Parafoil/WindEstimationScheme/WindEstimation.h>
#include <Parafoil/Wing/AutomaticWingAlgorithm.h>
#include <Parafoil/Wing/Guidance/EarlyManeuversGuidanceAlgorithm.h>
#include <algorithms/NAS/NASState.h>
#include <algorithms/PIController.h>
#include <algorithms/ReferenceValues.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <events/EventBroker.h>
#include <miosix.h>
#include <utils/AeroUtils/AeroUtils.h>
#include <utils/Debug.h>

#include <iostream>
#include <utils/ModuleManager/ModuleManager.hpp>
#include <vector>

// #include "parafoil-test-guidance-data.h"
#include "thread"

using namespace miosix;
using namespace Boardcore;
using namespace Parafoil;
using namespace std;
using namespace Eigen;
using namespace Common;

class MockWingController : public WingController
{
public:
    float getPsiRef() { return emGuidance.getPsiRef(); }
};

class MockWingAlgo : public AutomaticWingAlgorithm
{
public:
    MockWingAlgo(float kp, float ki, ServosList servo1, ServosList servo2,
                 GuidanceAlgorithm& guidance)
        : AutomaticWingAlgorithm(kp, ki, servo1, servo2, guidance)
    {
    }

    float fakeStep(NASState state, Vector2f windNED)
    {
        return this->algorithmStep(state, windNED);
    }
};

class MockNASController : public NASController, public HILSensor<NASState>
{
public:
    MockNASController(void* sensorData)
        : NASController(), HILSensor(1, sensorData)
    {
    }

    bool startModule() override { return true; }
    void update() override {}
    Boardcore::NASState getNasState() override { return lastSample; }

protected:
    NASState updateData() override
    {
        NASState tempData;
        HILConfig::SimulatorData::NAS* nas =
            reinterpret_cast<HILConfig::SimulatorData::NAS*>(sensorData);

        {
            miosix::PauseKernelLock pkLock;
            tempData.n  = nas->n;
            tempData.e  = nas->e;
            tempData.d  = nas->d;
            tempData.vn = nas->vn;
            tempData.ve = nas->ve;
            tempData.vd = nas->vd;
        }

        Boardcore::Logger::getInstance().log(tempData);

        return tempData;
    }
};

class MockEarlyManeuversGuidanceAlgorithm
    : public EarlyManeuversGuidanceAlgorithm
{
public:
    MockEarlyManeuversGuidanceAlgorithm() : EarlyManeuversGuidanceAlgorithm() {}

    float calculateTargetAngle(const Eigen::Vector3f& position,
                               Eigen::Vector2f& heading) override
    {
        psiRef = EarlyManeuversGuidanceAlgorithm::calculateTargetAngle(position,
                                                                       heading);
        return psiRef;
    }

    float getPsiRef() { return psiRef; }

private:
    float psiRef;
};

/**
 * @brief HILSensor implementation for the HIL test of Parafoil algorithms
 */
class HILSensors : public Sensors
{
public:
    HILSensors(TaskScheduler* scheduler) : scheduler(scheduler) {}

    bool startModule() override
    {
        // GPS
        {
            gps = new HILGps(HILConfig::N_DATA_GPS,
                             &Boardcore::ModuleManager::getInstance()
                                  .get<HIL>()
                                  ->simulator->getSensorData()
                                  ->gps);

            SensorInfo info("GPS_HIL", 1000 / HILConfig::GPS_FREQ);
            sensorMap.emplace(make_pair(gps, info));
        }

        // NAS
        {
            nas = static_cast<MockNASController*>(
                ModuleManager::getInstance().get<NASController>());

            SensorInfo info("NAS_HIL", HILConfig::SIMULATION_PERIOD);
            sensorMap.emplace(make_pair(nas, info));
        }

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

    HILGps* gps            = nullptr;
    MockNASController* nas = nullptr;
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
    Eigen::Vector2f TARGET{45.0018, 9.0038};  // 200, 300
    Eigen::Vector2f START{45, 9};

    TaskScheduler& scheduler = BoardScheduler::getInstance().getScheduler();
    EventBroker& eventBroker = EventBroker::getInstance();
    Sensors* sensors         = new HILSensors(&scheduler);
    Actuators* actuators     = new Actuators();
    HIL* hil                 = new HIL(usart2);
    WindEstimation* wind_estimation = new WindEstimation();
    NASController* nasController =
        new MockNASController(&hil->simulator->getSensorData()->nas);
    MockWingController* wingController = new MockWingController();

    printf("KP: %.3f, KI: %.3f\n", WingConfig::KP, WingConfig::KI);

    Eigen::Vector2f targetNED = Aeroutils::geodetic2NED(TARGET, START);

    // Insert the modules
    if (!modules.insert<HIL>(hil))
    {
        TRACE("Error inserting HIL\n");
    }

    if (!modules.insert<Sensors>(sensors))
    {
        TRACE("Error inserting Sensor\n");
    }

    if (!modules.insert<Actuators>(actuators))
    {
        TRACE("Error inserting Actuators\n");
    }

    if (!modules.insert<WindEstimation>(wind_estimation))
    {
        TRACE("Error inserting wind estimation\n");
    }

    if (!modules.insert<NASController>(nasController))
    {
        TRACE("Error inserting NASController\n");
    }

    if (!modules.insert<WingController>(wingController))
    {
        TRACE("Error inserting the WingController module\n");
    }

    // start the scheduler
    if (!eventBroker.start())
    {
        TRACE("Error starting the Event Broker\n");
    }

    if (!scheduler.start())
    {
        TRACE("Error starting the General Purpose Scheduler\n");
    }

    if (!hil->start())
    {
        TRACE("Error inserting HIL\n");
    }
    TRACE("inserting HIL\n");
    // Start the modules
    if (!ModuleManager::getInstance().get<Sensors>()->startModule())
    {
        TRACE("Error starting Sensors\n");
    }
    TRACE("starting Sensors\n");
    if (!ModuleManager::getInstance().get<Actuators>()->startModule())
    {
        TRACE("Error starting Actuators\n");
    }
    TRACE("starting Actuators\n");
    if (!ModuleManager::getInstance().get<WindEstimation>()->startModule())
    {
        TRACE("Error starting WindEstimation\n");
    }
    TRACE("starting WindEstimation\n");
    if (!ModuleManager::getInstance().get<NASController>()->startModule())
    {
        TRACE("Error starting NASController\n");
    }
    TRACE("starting NASController\n");

    if (!ModuleManager::getInstance().get<WingController>()->startModule())
    {
        TRACE("Error starting the WingController module\n");
    }
    TRACE("starting the WingController module\n");
    // BoardScheduler::getInstance().getScheduler().start();

    bool simulation_started = false;

    hil->flightPhasesManager->registerToFlightPhase(
        FlightPhases::SIMULATION_STARTED, [&]() { simulation_started = true; });

    // Waiting for the simulation to start, sending always a dummy packet
    printf("Waiting for simulation to start\n");
    while (!simulation_started)
    {
        auto wind = wind_estimation->getWindEstimationScheme();

        HILConfig::ActuatorData actuatorData{
            (wind_estimation->getStatus() ? 2.0f : 1.0f), wind[0], wind[1], 0,
            0};
        usart2.write(&actuatorData, sizeof(HILConfig::ActuatorData));
        Thread::sleep(100);
    }

    printf("SIMU STARTED\n");
    printf("CALIBRATE lat: %.3f, long: %.3f\n",
           sensors->getUbxGpsLastSample().latitude,
           sensors->getUbxGpsLastSample().longitude);
    nasController->calibrate();
    wingController->setTargetPosition(TARGET);
    wingController->addAlgorithm(WingConfig::SELECTED_ALGORITHM);
    wingController->postEvent(FLIGHT_WING_ALT_PASSED);

    // Adding task to send periodically to the simulator the wind estimated
    BoardScheduler::getInstance().getScheduler().addTask(
        [&]()
        {
            if (!wind_estimation->getStatus() &&
                hil->simulator->getSensorData()->wing.state > 1)
            {
                printf("CALIBRATION ENDED\n");

                wind_estimation->stopWindEstimationSchemeCalibration();
                wind_estimation->startWindEstimationScheme();
            }

            auto wind = wind_estimation->getWindEstimationScheme();

            // deltaA in degrees
            float result = actuators->getServoAngle(PARAFOIL_LEFT_SERVO) -
                           actuators->getServoAngle(PARAFOIL_RIGHT_SERVO);

            // Convert back from degrees in rad (what the simulator expects)
            result = -result * Constants::PI / 180.0f;

            float psiRef = wingController->getPsiRef();
            // printf("deltaA: %+.3f, psiRef: %+.3f\n", result, psiRef);

            HILConfig::ActuatorData actuatorData{
                (wind_estimation->getStatus() ? 2.0f : 1.0f), wind[0], wind[1],
                psiRef, result};

            // Actually sending the feedback to the simulator
            modules.get<HIL>()->send(actuatorData);

            // UBXGPSData gpsData = sensors->getUbxGpsLastSample();
            // TRACE("gpsP: [%f, %f, %f]\n", gpsData.latitude,
            // gpsData.longitude,
            //       gpsData.height);
            // TRACE("gpsV: [%f, %f, %f]\n", gpsData.velocityNorth,
            //       gpsData.velocityEast, gpsData.velocityDown);
            // TRACE("NAS: P[%f,%f,%f] V[%f,%f,%f]\n", nasState.n, nasState.e,
            //       nasState.d, nasState.vn, nasState.ve, nasState.vd);
            // actuatorData.print();
            // printf("\n");
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
