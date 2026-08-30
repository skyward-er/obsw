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

#include <RIGv3/Actuators/Actuators.h>
#include <RIGv3/CanHandler/CanHandler.h>
#include <RIGv3/Configs/RadioConfig.h>
#include <RIGv3/Radio/Radio.h>
#include <RIGv3/Sensors/Sensors.h>

#include "RIGv3/Configs/ValveSequenceControllerConfig.h"

using namespace std::chrono;
using namespace Boardcore;
using namespace Common;

namespace RIGv3
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
            uint16_t wiggleResult = wiggleValves();
            getModule<Radio>()->enqueueWiggleResultTm(wiggleResult,
                                                      lastRequestId);
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

void ValveSequenceController::requestAsyncAutomaticWiggle(uint8_t requestId)
{
    lastRequestId = requestId;
    EventBroker::getInstance().post(WIGGLE_ALL_VALVES, TOPIC_VALVE_SEQUENCE);
}

void ValveSequenceController::closeValves()
{
    getModule<Actuators>()->closeValve(PRZ_FILLING_VALVE);
    getModule<Actuators>()->closeValve(PRZ_RELEASE_VALVE);
    getModule<Actuators>()->closeValve(OX_FILLING_VALVE);
    getModule<Actuators>()->closeValve(OX_RELEASE_VALVE);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    getModule<Actuators>()->closeValve(OX_VENTING_VALVE);
    getModule<Actuators>()->closeValve(FUEL_VENTING_VALVE);
    getModule<Actuators>()->closeValve(OX_DETACH_SERVO);
    getModule<Actuators>()->closeValve(FUEL_DETACH_SERVO);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    getModule<Actuators>()->closeValve(PURGE_VALVE);
    getModule<Actuators>()->closeValve(IGNITION_FUEL_VALVE);
    getModule<Actuators>()->closeValve(IGNITION_OX_VALVE);
    getModule<Actuators>()->closeValve(PRZ_OX_VALVE);
    EventBroker::getInstance().post(Common::Events::EREG_CLOSE, TOPIC_EREG_OX);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    getModule<Actuators>()->closeValve(PRZ_FUEL_VALVE);
    EventBroker::getInstance().post(Common::Events::EREG_CLOSE,
                                    TOPIC_EREG_FUEL);
    getModule<Actuators>()->closeValve(MAIN_OX_VALVE);
    getModule<Actuators>()->closeValve(MAIN_FUEL_VALVE);
}

uint16_t ValveSequenceController::wiggleValves()
{
    auto isValveOpen =
        [&](auto getPosition, uint8_t bit, uint8_t openingThreshold)
    {
        bool isOpen = static_cast<uint8_t>(getPosition()) > openingThreshold;
        return isOpen ? static_cast<uint8_t>(1 << bit) : 0;
    };

    auto isValveClosed =
        [&](auto getPosition, uint8_t bit, uint8_t closedThreshold)
    {
        bool isClosed = static_cast<uint8_t>(getPosition()) < closedThreshold;
        return isClosed ? static_cast<uint8_t>(0xFF)
                        : static_cast<uint8_t>(~(1 << bit));
    };

    uint8_t wiggleMapRIG   = 0;
    uint8_t wiggleMapMotor = 0;

    getModule<Actuators>()->openValveWithTime(MAIN_OX_VALVE, 6500);
    getModule<Actuators>()->openValveWithTime(MAIN_FUEL_VALVE, 6500);
    getModule<Actuators>()->openValveWithTime(PRZ_OX_VALVE, 6500);
    getModule<Actuators>()->openValveWithTime(PRZ_FUEL_VALVE, 6500);

    Thread::sleep(Config::VALVE_WIGGLE_DELAY);

    wiggleMapRIG |= isValveOpen(
        [&]() { return getModule<Sensors>()->getMainOxPosition().position; },
        Config::ValveBit::MAIN_OX_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_MAIN_OX);
    wiggleMapRIG |= isValveOpen(
        [&]() { return getModule<Sensors>()->getMainFuelPosition().position; },
        Config::ValveBit::MAIN_FUEL_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_MAIN_FUEL);
    wiggleMapRIG |= isValveOpen(
        [&]() { return getModule<Sensors>()->getOxRegPosition().position; },
        Config::ValveBit::PRZ_OX_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_PRZ_OX);
    wiggleMapRIG |= isValveOpen(
        [&]() { return getModule<Sensors>()->getFuelRegPosition().position; },
        Config::ValveBit::PRZ_FUEL_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_PRZ_FUEL);

    wiggleMapMotor |= isValveOpen(
        [&]()
        { return getModule<MotorStatus>()->lockData()->mainOxValvePosition; },
        Config::ValveBit::MAIN_OX_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_MAIN_OX);
    wiggleMapMotor |= isValveOpen(
        [&]()
        { return getModule<MotorStatus>()->lockData()->mainFuelValvePosition; },
        Config::ValveBit::MAIN_FUEL_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_MAIN_FUEL);
    wiggleMapMotor |= isValveOpen(
        [&]()
        { return getModule<MotorStatus>()->lockData()->przOxValvePosition; },
        Config::ValveBit::PRZ_OX_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_PRZ_OX);
    wiggleMapMotor |= isValveOpen(
        [&]()
        { return getModule<MotorStatus>()->lockData()->przFuelValvePosition; },
        Config::ValveBit::PRZ_FUEL_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_PRZ_FUEL);

    getModule<Actuators>()->closeValve(MAIN_OX_VALVE);
    getModule<Actuators>()->closeValve(MAIN_FUEL_VALVE);
    getModule<Actuators>()->closeValve(PRZ_OX_VALVE);
    getModule<Actuators>()->closeValve(PRZ_FUEL_VALVE);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    wiggleMapRIG &= isValveClosed(
        [&]() { return getModule<Sensors>()->getMainOxPosition().position; },
        Config::ValveBit::MAIN_OX_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_MAIN_OX);
    wiggleMapRIG &= isValveClosed(
        [&]() { return getModule<Sensors>()->getMainFuelPosition().position; },
        Config::ValveBit::MAIN_FUEL_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_MAIN_FUEL);
    wiggleMapRIG &= isValveClosed(
        [&]() { return getModule<Sensors>()->getOxRegPosition().position; },
        Config::ValveBit::PRZ_OX_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_PRZ_OX);
    wiggleMapRIG &= isValveClosed(
        [&]() { return getModule<Sensors>()->getFuelRegPosition().position; },
        Config::ValveBit::PRZ_FUEL_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_PRZ_FUEL);
    wiggleMapMotor &= isValveClosed(
        [&]()
        { return getModule<MotorStatus>()->lockData()->mainOxValvePosition; },
        Config::ValveBit::MAIN_OX_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_MAIN_OX);
    wiggleMapMotor &= isValveClosed(
        [&]()
        { return getModule<MotorStatus>()->lockData()->mainFuelValvePosition; },
        Config::ValveBit::MAIN_FUEL_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_MAIN_FUEL);
    wiggleMapMotor &= isValveClosed(
        [&]()
        { return getModule<MotorStatus>()->lockData()->przOxValvePosition; },
        Config::ValveBit::PRZ_OX_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_PRZ_OX);
    wiggleMapMotor &= isValveClosed(
        [&]()
        { return getModule<MotorStatus>()->lockData()->przFuelValvePosition; },
        Config::ValveBit::PRZ_FUEL_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_PRZ_FUEL);

    getModule<Actuators>()->openValveWithTime(OX_VENTING_VALVE, 6500);
    getModule<Actuators>()->openValveWithTime(FUEL_VENTING_VALVE, 6500);

    Thread::sleep(Config::VALVE_WIGGLE_DELAY);

    wiggleMapMotor |= isValveOpen(
        [&]()
        {
            return getModule<MotorStatus>()->lockData()->oxVentingValvePosition;
        },
        Config::ValveBit::OX_VENTING_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_OX_VENTING);
    wiggleMapMotor |= isValveOpen(
        [&]()
        {
            return getModule<MotorStatus>()
                ->lockData()
                ->fuelVentingValvePosition;
        },
        Config::ValveBit::FUEL_VENTING_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_FUEL_VENTING);

    getModule<Actuators>()->closeValve(OX_VENTING_VALVE);
    getModule<Actuators>()->closeValve(FUEL_VENTING_VALVE);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    wiggleMapMotor &= isValveClosed(
        [&]()
        {
            return getModule<MotorStatus>()->lockData()->oxVentingValvePosition;
        },
        Config::ValveBit::OX_VENTING_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_OX_VENTING);
    wiggleMapMotor &= isValveClosed(
        [&]()
        {
            return getModule<MotorStatus>()
                ->lockData()
                ->fuelVentingValvePosition;
        },
        Config::ValveBit::FUEL_VENTING_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_FUEL_VENTING);

    return static_cast<uint16_t>(wiggleMapRIG |
                                 (static_cast<uint16_t>(wiggleMapMotor) << 8));
}
}  // namespace RIGv3
