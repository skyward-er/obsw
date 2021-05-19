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

#include <iostream>
#include <stdexcept>

#include "SensorManagerData.h"
#include "System/StackLogger.h"
#include "events/EventBroker.h"
#include "events/Events.h"
#include "events/Topics.h"

#ifdef USE_MOCK_SENSORS
#include "Sensors/Test/TestSensor.h"
#include "Sensors/Test/MockGPS.h"
#include "Sensors/Test/MockPressureSensor.h"
#endif

#include "Common.h"
#include "interfaces-impl/hwmapping.h"

using miosix::FastMutex;
using miosix::Lock;

using namespace miosix;

namespace DeathStackBoard
{

SensorManager::SensorManager()
    : FSM(&SensorManager::state_idle),
      logger(*LoggerService::getInstance())
{
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);
}

SensorManager::~SensorManager()
{
    sEventBroker->unsubscribe(this);
}

bool SensorManager::start()
{
    // Start the parent FSM
    return FSM<SensorManager>::start();
}

void SensorManager::state_idle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            enable_sensor_logging = false;

            status.timestamp = TimestampTimer::getTimestamp();
            status.state     = SensorManagerState::IDLE;
            logger.log(status);

            TRACE("[SM] Entering state_idle\n");

            StackLogger::getInstance()->updateStack(THID_SENSOR_MANAGER);

            break;
        case EV_EXIT:
            TRACE("[SM] Exiting stateIdle\n");

            break;

        // Perform the transition in both cases
        case EV_TC_START_SENSOR_LOGGING:
        case EV_ARMED:
            transition(&SensorManager::state_active);
            break;

        default:
            break;
    }
}

void SensorManager::state_calibration(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            enable_sensor_logging = false;

            status.timestamp = TimestampTimer::getTimestamp();
            status.state     = SensorManagerState::IDLE;
            logger.log(status);

            TRACE("[SM] Entering state_calibration\n");

            StackLogger::getInstance()->updateStack(THID_SENSOR_MANAGER);

            break;
        case EV_EXIT:
            TRACE("[SM] Exiting state_calibration\n");

            break;

        // Perform the transition in both cases
        case EV_TC_START_SENSOR_LOGGING:
        case EV_ARMED:
            transition(&SensorManager::state_active);
            break;

        default:
            break;
    }
}

void SensorManager::state_active(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            enable_sensor_logging = true;
            status.timestamp      = TimestampTimer::getTimestamp();
            status.state          = SensorManagerState::ACTIVE;
            logger.log(status);

            TRACE("[SM] Entering state_active\n");

            StackLogger::getInstance()->updateStack(THID_SENSOR_MANAGER);

            break;
        case EV_EXIT:

            TRACE("[SM] Exiting state_active\n");

            break;
        // Go back to idle in both cases
        case EV_TC_STOP_SENSOR_LOGGING:
        case EV_LANDED:
            transition(&SensorManager::state_idle);
            break;

        default:
            break;
    }
}

}  // namespace DeathStackBoard
