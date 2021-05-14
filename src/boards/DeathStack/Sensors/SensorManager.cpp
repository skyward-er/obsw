/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SensorManager.h"

#include <math/Stats.h>

#include "SensorManagerData.h"
#include "System/StackLogger.h"
#include "events/EventBroker.h"
#include "events/Events.h"
#include "events/Topics.h"

#ifdef USE_MOCK_SENSORS
#include "Sensors/Mock/TestSensor.h"
#include "Sensors/Mock/MockGPS.h"
#include "Sensors/Mock/MockPressureSensor.h"
#endif

#include "Common.h"
#include "interfaces-impl/hwmapping.h"

using miosix::FastMutex;
using miosix::Lock;

using namespace miosix;

namespace DeathStackBoard
{

SensorManager::SensorManager()
    : FSM(&SensorManager::stateIdle), scheduler(),
      logger(*LoggerService::getInstance())
{
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);

    memset(&sensor_status, 0, sizeof(sensor_status));

    initSensors();

    initScheduler();
}

SensorManager::~SensorManager()
{
    sEventBroker->unsubscribe(this);
    scheduler.stop();
}

bool SensorManager::start()
{
    // Start the parent FSM
    bool ok = FSM<SensorManager>::start();

    // Start the scheduler
    ok = ok && scheduler.start();

    return ok;
}

void SensorManager::initSensors()
{

#ifdef USE_MOCK_SENSORS
    mock_pressure_sensor = new MockPressureSensor();
    mock_gps             = new MockGPS();
#endif

    TRACE("[SM] Sensor init done\n");
}

void SensorManager::initScheduler()
{

    TRACE("[SM] Scheduler initialization complete\n");
}

void SensorManager::stateIdle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            enable_sensor_logging = false;

            status.timestamp = TimestampTimer::getTimestamp();
            status.state     = SensorManagerState::IDLE;
            logger.log(status);

            TRACE("[SM] Entering stateIdle\n");

            StackLogger::getInstance()->updateStack(THID_SENSOR_MANAGER);

            break;
        case EV_EXIT:
            TRACE("[SM] Exiting stateIdle\n");

            break;

        // Perform the transition in both cases
        case EV_TC_START_SENSOR_LOGGING:
        case EV_ARMED:
            transition(&SensorManager::stateLogging);
            break;

        default:
            break;
    }
}

void SensorManager::stateLogging(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            enable_sensor_logging = true;
            status.timestamp      = TimestampTimer::getTimestamp();
            status.state          = SensorManagerState::LOGGING;
            logger.log(status);

            TRACE("[SM] Entering stateLogging\n");

            StackLogger::getInstance()->updateStack(THID_SENSOR_MANAGER);

            break;
        case EV_EXIT:

            TRACE("[SM] Exiting stateLogging\n");

            break;
#ifdef USE_MOCK_SENSORS
        // Signal to the mock pressure sensor that we have liftoff in order
        // to start simulating flight pressures
        case EV_LIFTOFF:
            mock_pressure_sensor->signalLiftoff();
            mock_gps->signalLiftoff();
            break;
#endif
        // Go back to idle in both cases
        case EV_TC_STOP_SENSOR_LOGGING:
        case EV_LANDED:
            transition(&SensorManager::stateIdle);
            break;

        default:
            break;
    }
}

void SensorManager::onSimple20HZCallback()
{

#ifdef USE_MOCK_SENSORS
    PressureData p_data = mock_pressure_sensor->getLastSample();
#endif

    if (enable_sensor_logging)
    {
        // logger.log(...);
    }
}

void SensorManager::onSimple50HZCallback()
{
    if (enable_sensor_logging)
    {
        // Since sampling both temps & pressure on the ms5803 takes two calls of
        // onSimpleUpdate(), log only once every 2
        /*if (pressure_ms5803->getState() ==
            MS580301BA07Type::STATE_SAMPLED_PRESSURE)
        {
            logger.log(pressure_ms5803->getData());
        }*/
    }
}

void SensorManager::onSimple100HZCallback() {}

void SensorManager::onSimple250HZCallback() {}

void SensorManager::onGPSCallback()
{

#ifdef USE_MOCK_SENSORS
    GPSData data = mock_gps->getLastSample();
#endif

    if (enable_sensor_logging)
    {
        // logger.log(data);
    }
}

}  // namespace DeathStackBoard
