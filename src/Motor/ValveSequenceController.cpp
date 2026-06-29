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
        case WIGGLE_ALL_VALVES:
        {
            uint8_t wiggleResult = wiggleValves();
            getModule<CanHandler>()->sendWiggleResult(wiggleResult);
            break;
        }

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
    getModule<Actuators>()->closeValve(OX_VENTING_VALVE);
    getModule<Actuators>()->closeValve(FUEL_VENTING_VALVE);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    getModule<Actuators>()->closeValve(IGNITION_OX_VALVE);
    getModule<Actuators>()->closeValve(IGNITION_FUEL_VALVE);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    EventBroker::getInstance().post(Common::Events::EREG_CLOSE, TOPIC_EREG_OX);
    EventBroker::getInstance().post(Common::Events::EREG_CLOSE,
                                    TOPIC_EREG_FUEL);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    getModule<Actuators>()->closeValve(MAIN_OX_VALVE);
    getModule<Actuators>()->closeValve(MAIN_FUEL_VALVE);
}

uint8_t ValveSequenceController::wiggleValves()
{
    auto checkValve = [&](auto getPosition, uint8_t bit, auto openingThreshold,
                          auto closedThreshold)
    {
        Thread::sleep(Config::VALVE_WIGGLE_DELAY);

        bool isOpen = getPosition().position > openingThreshold;

        Thread::sleep(Config::VALVE_WIGGLE_DELAY);

        bool isClosed = getPosition().position < closedThreshold;

        return (isOpen && isClosed) ? static_cast<uint8_t>(1 << bit) : 0;
    };

    uint8_t wiggleMap = 0;

    getModule<Actuators>()->wiggleValve(MAIN_OX_VALVE);
    wiggleMap |=
        checkValve([&]() { return getModule<Sensors>()->getMainOxPosition(); },
                   0, Config::VALVE_CLOSED_THRESHOLD_MAIN_OX,
                   Config::VALVE_CLOSED_THRESHOLD_MAIN_OX);

    getModule<Actuators>()->wiggleValve(MAIN_FUEL_VALVE);
    wiggleMap |= checkValve(
        [&]() { return getModule<Sensors>()->getMainFuelPosition(); }, 1,
        Config::VALVE_CLOSED_THRESHOLD_MAIN_FUEL,
        Config::VALVE_CLOSED_THRESHOLD_MAIN_FUEL);

    getModule<Actuators>()->wiggleValve(PRZ_OX_VALVE);
    wiggleMap |=
        checkValve([&]() { return getModule<Sensors>()->getPrzOxPosition(); },
                   2, Config::VALVE_CLOSED_THRESHOLD_PRZ_OX,
                   Config::VALVE_CLOSED_THRESHOLD_PRZ_OX);

    getModule<Actuators>()->wiggleValve(PRZ_FUEL_VALVE);
    wiggleMap |=
        checkValve([&]() { return getModule<Sensors>()->getPrzFuelPosition(); },
                   3, Config::VALVE_CLOSED_THRESHOLD_PRZ_FUEL,
                   Config::VALVE_CLOSED_THRESHOLD_PRZ_FUEL);

    getModule<Actuators>()->wiggleValve(OX_VENTING_VALVE);
    wiggleMap |= checkValve(
        [&]() { return getModule<Sensors>()->getVentingOxPosition(); }, 4,
        Config::VALVE_OPENING_THRESHOLD_OX_VENTING,
        Config::VALVE_CLOSED_THRESHOLD_OX_VENTING);

    getModule<Actuators>()->wiggleValve(FUEL_VENTING_VALVE);
    wiggleMap |= checkValve(
        [&]() { return getModule<Sensors>()->getVentingFuelPosition(); }, 5,
        Config::VALVE_OPENING_THRESHOLD_FUEL_VENTING,
        Config::VALVE_CLOSED_THRESHOLD_FUEL_VENTING);

    return wiggleMap;
}
}  // namespace Motor
