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

#include "Motor/Configs/ValveSequenceControllerConfig.h"

using namespace std::chrono;
using namespace Boardcore;
using namespace Common;

namespace Motor
{

ValveSequenceController::ValveSequenceController() {}

void ValveSequenceController::closeValves()
{
    getModule<Actuators>()->closeValve(OX_VENTING_VALVE);
    getModule<Actuators>()->closeValve(FUEL_VENTING_VALVE);

    Thread::sleep(Config::VALVE_CLOSING_TIME);

    getModule<Actuators>()->closeValve(IGNITION_OX_VALVE);
    getModule<Actuators>()->closeValve(IGNITION_FUEL_VALVE);

    Thread::sleep(Config::VALVE_CLOSING_TIME);

    EventBroker::getInstance().post(Common::Events::EREG_CLOSE, TOPIC_EREG_OX);
    EventBroker::getInstance().post(Common::Events::EREG_CLOSE,
                                    TOPIC_EREG_FUEL);

    Thread::sleep(Config::VALVE_CLOSING_TIME);

    getModule<Actuators>()->closeValve(MAIN_OX_VALVE);
    getModule<Actuators>()->closeValve(MAIN_FUEL_VALVE);
}

uint8_t ValveSequenceController::wiggleValves()
{
    auto checkValve = [&](auto getPosition, uint8_t bit)
    {
        Thread::sleep(Config::VALVE_OPENING_TIME);

        bool isOpen = getPosition().position > Config::VALVE_OPENING_THRESHOLD;

        Thread::sleep(Config::VALVE_OPENING_TIME);

        bool isClosed = getPosition().position < Config::VALVE_CLOSED_THRESHOLD;

        return (isOpen && isClosed) ? static_cast<uint8_t>(1 << bit) : 0;
    };

    uint8_t wiggleMap = 0;

    getModule<Actuators>()->wiggleValve(MAIN_OX_VALVE);
    wiggleMap |= checkValve(
        [&]() { return getModule<Sensors>()->getMainOxPosition(); }, 0);

    getModule<Actuators>()->wiggleValve(MAIN_FUEL_VALVE);
    wiggleMap |= checkValve(
        [&]() { return getModule<Sensors>()->getMainFuelPosition(); }, 1);

    getModule<Actuators>()->wiggleValve(PRZ_OX_VALVE);
    wiggleMap |= checkValve(
        [&]() { return getModule<Sensors>()->getPrzOxPosition(); }, 2);

    getModule<Actuators>()->wiggleValve(PRZ_FUEL_VALVE);
    wiggleMap |= checkValve(
        [&]() { return getModule<Sensors>()->getPrzFuelPosition(); }, 3);

    getModule<Actuators>()->wiggleValve(OX_VENTING_VALVE);
    wiggleMap |= checkValve(
        [&]() { return getModule<Sensors>()->getVentingOxPosition(); }, 4);

    getModule<Actuators>()->wiggleValve(FUEL_VENTING_VALVE);
    wiggleMap |= checkValve(
        [&]() { return getModule<Sensors>()->getVentingFuelPosition(); }, 5);

    return wiggleMap;
}
}  // namespace Motor
