/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Riccardo Sironi
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

#include "ValveSequenceController.h"

#include <Motor/Actuators/Actuators.h>
#include <Motor/CanHandler/CanHandler.h>
#include <Motor/Sensors/Sensors.h>
#include <events/EventBroker.h>

#include "Motor/Configs/ValveSequenceControllerConfig.h"

using namespace Common;
using namespace Boardcore;
using namespace miosix;
using namespace std::chrono;

namespace Motor
{

ValveSequenceController::ValveSequenceController(unsigned int stacksize,
                                                 miosix::Priority priority)
    : EventHandler(stacksize, priority)
{
    EventBroker::getInstance().subscribe(this, TOPIC_VALVE_SEQUENCE);
}

void ValveSequenceController::handleEvent(const Event& ev)
{
    switch (ev)
    {
        case CLOSE_ALL_VALVES:
        {
            closeValves();
            break;
        }

        default:
            break;
    }
}

void ValveSequenceController::closeValves()
{
    LOG_INFO(logger, "Closing venting valves");
    getModule<Actuators>()->closeValve(OX_VENTING_VALVE);
    getModule<Actuators>()->closeValve(FUEL_VENTING_VALVE);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    LOG_INFO(logger, "Closing ignition valves");
    getModule<Actuators>()->closeValve(IGNITION_OX_VALVE);
    getModule<Actuators>()->closeValve(IGNITION_FUEL_VALVE);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    LOG_INFO(logger, "Closing Ereg (PRZ) valves");
    EventBroker::getInstance().post(Common::Events::EREG_CLOSE, TOPIC_EREG_OX);
    getModule<Actuators>()->closeValve(PRZ_OX_VALVE);
    EventBroker::getInstance().post(Common::Events::EREG_CLOSE,
                                    TOPIC_EREG_FUEL);
    getModule<Actuators>()->closeValve(PRZ_FUEL_VALVE);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    LOG_INFO(logger, "Closing Main valves");
    getModule<Actuators>()->closeValve(MAIN_OX_VALVE);
    getModule<Actuators>()->closeValve(MAIN_FUEL_VALVE);

    LOG_INFO(logger, "Closed all valves");
}

}  // namespace Motor
