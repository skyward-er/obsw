/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include "PinHandler.h"

#include <Main/Configs/PinHandlerConfig.h>
#include <common/Events.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>

#include "PinData.h"

using namespace Main;
using namespace Boardcore;
using namespace Common;
using namespace miosix;

PinHandler::PinHandler()
{
    PinObserver::getInstance().registerPinCallback(
        sense::detachRamp::getPin(),
        [this](PinTransition transition) { onRampPinTransition(transition); },
        Config::PinHandler::RAMP_PIN_THRESHOLD);

    PinObserver::getInstance().registerPinCallback(
        sense::detachMain::getPin(),
        [this](PinTransition transition)
        { onDetachMainTransition(transition); },
        Config::PinHandler::MAIN_DETACH_PIN_THRESHOLD);

    PinObserver::getInstance().registerPinCallback(
        sense::detachPayload::getPin(),
        [this](PinTransition transition)
        { onDetachPayloadTransition(transition); },
        Config::PinHandler::PAYLOAD_DETACH_PIN_THRESHOLD);

    PinObserver::getInstance().registerPinCallback(
        sense::expulsionSense::getPin(),
        [this](PinTransition transition)
        { onExpulsionSenseTransition(transition); },
        Config::PinHandler::EXPULSION_PIN_THRESHOLD);
}

bool PinHandler::start() { return PinObserver::getInstance().start(); }

void PinHandler::onRampPinTransition(PinTransition transition)
{
    LOG_INFO(logger, "onRampPinTransition {}", static_cast<int>(transition));

    if (transition == Config::PinHandler::RAMP_PIN_TRIGGER)
    {
        EventBroker::getInstance().post(FLIGHT_LAUNCH_PIN_DETACHED,
                                        TOPIC_FLIGHT);
    }
}

void PinHandler::onDetachMainTransition(PinTransition transition)
{
    LOG_INFO(logger, "onDetachMainTransition {}", static_cast<int>(transition));
}

void PinHandler::onDetachPayloadTransition(PinTransition transition)
{
    LOG_INFO(logger, "onDetachPayloadTransition {}",
             static_cast<int>(transition));
}

void PinHandler::onExpulsionSenseTransition(PinTransition transition)
{
    LOG_INFO(logger, "onExpulsionSenseTransition {}",
             static_cast<int>(transition));
}

PinData PinHandler::getPinData(PinList pin)
{
    switch (pin)
    {
        case PinList::RAMP_PIN:
            return PinObserver::getInstance().getPinData(
                sense::detachRamp::getPin());
        case PinList::DETACH_MAIN_PIN:
            return PinObserver::getInstance().getPinData(
                sense::detachMain::getPin());
        case PinList::DETACH_PAYLOAD_PIN:
            return PinObserver::getInstance().getPinData(
                sense::detachPayload::getPin());
        case PinList::EXPULSION_SENSE:
            return PinObserver::getInstance().getPinData(
                sense::expulsionSense::getPin());
        default:
            return PinData{};
    }
}
