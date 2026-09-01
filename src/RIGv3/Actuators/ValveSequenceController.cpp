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
            wiggleValves();
            getModule<Radio>()->enqueueWiggleResultTm(
                wiggleResult.mainOxSuccess, wiggleResult.mainFuelSuccess,
                wiggleResult.przOxSuccess, wiggleResult.przFuelSuccess,
                wiggleResult.oxVentingSuccess, wiggleResult.fuelVentingSuccess,
                wiggleResult.prz3WaySuccess, wiggleResult.przFillingSuccess,
                wiggleResult.przReleaseSuccess, wiggleResult.oxFillingSuccess,
                wiggleResult.oxReleaseSuccess, lastRequestId);
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

void ValveSequenceController::wiggleValves()
{
    auto isValveOpen =
        [&](auto getPosition, uint8_t bit, uint8_t openingThreshold)
    { return static_cast<uint8_t>(getPosition()) > openingThreshold; };

    auto isValveClosed =
        [&](auto getPosition, uint8_t bit, uint8_t closedThreshold)
    { return static_cast<uint8_t>(getPosition()) < closedThreshold; };

    getModule<Actuators>()->openValveWithTime(MAIN_OX_VALVE, 6500);
    getModule<Actuators>()->openValveWithTime(MAIN_FUEL_VALVE, 6500);

    Thread::sleep(Config::VALVE_WIGGLE_DELAY);

    wiggleResult.mainOxSuccess = isValveOpen(
        [&]()
        { return getModule<MotorStatus>()->lockData()->mainOxValvePosition; },
        Config::MotorValveBit::MAIN_OX_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_MAIN_OX);
    wiggleResult.mainFuelSuccess = isValveOpen(
        [&]()
        { return getModule<MotorStatus>()->lockData()->mainFuelValvePosition; },
        Config::MotorValveBit::MAIN_FUEL_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_MAIN_FUEL);

    getModule<Actuators>()->closeValve(MAIN_OX_VALVE);
    getModule<Actuators>()->closeValve(MAIN_FUEL_VALVE);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    wiggleResult.mainOxSuccess &= isValveClosed(
        [&]()
        { return getModule<MotorStatus>()->lockData()->mainOxValvePosition; },
        Config::MotorValveBit::MAIN_OX_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_MAIN_OX);
    wiggleResult.mainFuelSuccess &= isValveClosed(
        [&]()
        { return getModule<MotorStatus>()->lockData()->mainFuelValvePosition; },
        Config::MotorValveBit::MAIN_FUEL_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_MAIN_FUEL);

    getModule<Actuators>()->openValveWithTime(PRZ_OX_VALVE, 6500);
    getModule<Actuators>()->openValveWithTime(PRZ_FUEL_VALVE, 6500);

    Thread::sleep(Config::VALVE_WIGGLE_DELAY);

    wiggleResult.przOxSuccess = isValveOpen(
        [&]()
        { return getModule<MotorStatus>()->lockData()->przOxValvePosition; },
        Config::MotorValveBit::PRZ_OX_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_PRZ_OX);
    wiggleResult.przFuelSuccess = isValveOpen(
        [&]()
        { return getModule<MotorStatus>()->lockData()->przFuelValvePosition; },
        Config::MotorValveBit::PRZ_FUEL_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_PRZ_FUEL);

    getModule<Actuators>()->closeValve(PRZ_OX_VALVE);
    getModule<Actuators>()->closeValve(PRZ_FUEL_VALVE);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    wiggleResult.przOxSuccess &= isValveClosed(
        [&]()
        { return getModule<MotorStatus>()->lockData()->przOxValvePosition; },
        Config::MotorValveBit::PRZ_OX_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_PRZ_OX);
    wiggleResult.przFuelSuccess &= isValveClosed(
        [&]()
        { return getModule<MotorStatus>()->lockData()->przFuelValvePosition; },
        Config::MotorValveBit::PRZ_FUEL_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_PRZ_FUEL);

    getModule<Actuators>()->openValveWithTime(OX_VENTING_VALVE, 6500);
    getModule<Actuators>()->openValveWithTime(FUEL_VENTING_VALVE, 6500);

    Thread::sleep(Config::VALVE_WIGGLE_DELAY);

    wiggleResult.oxVentingSuccess = isValveOpen(
        [&]()
        {
            return getModule<MotorStatus>()->lockData()->oxVentingValvePosition;
        },
        Config::MotorValveBit::OX_VENTING_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_OX_VENTING);
    wiggleResult.fuelVentingSuccess = isValveOpen(
        [&]()
        {
            return getModule<MotorStatus>()
                ->lockData()
                ->fuelVentingValvePosition;
        },
        Config::MotorValveBit::FUEL_VENTING_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_FUEL_VENTING);

    getModule<Actuators>()->closeValve(OX_VENTING_VALVE);
    getModule<Actuators>()->closeValve(FUEL_VENTING_VALVE);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    wiggleResult.oxVentingSuccess &= isValveClosed(
        [&]()
        {
            return getModule<MotorStatus>()->lockData()->oxVentingValvePosition;
        },
        Config::MotorValveBit::OX_VENTING_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_OX_VENTING);
    wiggleResult.fuelVentingSuccess &= isValveClosed(
        [&]()
        {
            return getModule<MotorStatus>()
                ->lockData()
                ->fuelVentingValvePosition;
        },
        Config::MotorValveBit::FUEL_VENTING_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_FUEL_VENTING);

    // Wiggle RIG Valves

    getModule<Actuators>()->set3wayValveState(true);
    getModule<Actuators>()->openValveWithTime(PRZ_FILLING_VALVE, 6500);
    getModule<Actuators>()->openValveWithTime(PRZ_RELEASE_VALVE, 6500);

    Thread::sleep(Config::VALVE_WIGGLE_DELAY);

    wiggleResult.prz3WaySuccess = isValveOpen(
        [&]() { return getModule<Sensors>()->getPrz3WayPosition().position; },
        Config::RIGValveBit::PRZ_3WAY_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_PRZ_3WAY);
    wiggleResult.przFillingSuccess = isValveOpen(
        [&]()
        { return getModule<Sensors>()->getPrzFillingPosition().position; },
        Config::RIGValveBit::PRZ_FILLING_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_PRZ_FILLING);
    wiggleResult.przReleaseSuccess = isValveOpen(
        [&]()
        { return getModule<Sensors>()->getPrzReleasePosition().position; },
        Config::RIGValveBit::PRZ_RELEASE_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_PRZ_RELEASE);

    getModule<Actuators>()->set3wayValveState(false);
    getModule<Actuators>()->closeValve(PRZ_FILLING_VALVE);
    getModule<Actuators>()->closeValve(PRZ_RELEASE_VALVE);

    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    wiggleResult.prz3WaySuccess &= isValveClosed(
        [&]() { return getModule<Sensors>()->getPrz3WayPosition().position; },
        Config::RIGValveBit::PRZ_3WAY_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_PRZ_3WAY);
    wiggleResult.przFillingSuccess &= isValveClosed(
        [&]()
        { return getModule<Sensors>()->getPrzFillingPosition().position; },
        Config::RIGValveBit::PRZ_FILLING_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_PRZ_FILLING);
    wiggleResult.przReleaseSuccess &= isValveClosed(
        [&]()
        { return getModule<Sensors>()->getPrzReleasePosition().position; },
        Config::RIGValveBit::PRZ_RELEASE_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_PRZ_RELEASE);

    getModule<Actuators>()->openValveWithTime(OX_FILLING_VALVE, 6500);
    getModule<Actuators>()->openValveWithTime(OX_RELEASE_VALVE, 6500);

    Thread::sleep(Config::VALVE_WIGGLE_DELAY);

    wiggleResult.oxFillingSuccess = isValveOpen(
        [&]() { return getModule<Sensors>()->getOxFillingPosition().position; },
        Config::RIGValveBit::OX_FILLING_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_OX_FILLING);
    wiggleResult.oxReleaseSuccess = isValveOpen(
        [&]() { return getModule<Sensors>()->getOxReleasePosition().position; },
        Config::RIGValveBit::OX_RELEASE_VALVE_BIT,
        Config::VALVE_OPENING_THRESHOLD_OX_RELEASE);

    getModule<Actuators>()->closeValve(OX_FILLING_VALVE);
    getModule<Actuators>()->closeValve(OX_RELEASE_VALVE);
    Thread::sleep(Config::VALVE_CLOSING_DELAY);

    wiggleResult.oxFillingSuccess &= isValveClosed(
        [&]() { return getModule<Sensors>()->getOxFillingPosition().position; },
        Config::RIGValveBit::OX_FILLING_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_OX_FILLING);
    wiggleResult.oxReleaseSuccess &= isValveClosed(
        [&]() { return getModule<Sensors>()->getOxReleasePosition().position; },
        Config::RIGValveBit::OX_RELEASE_VALVE_BIT,
        Config::VALVE_CLOSED_THRESHOLD_OX_RELEASE);
}
}  // namespace RIGv3
