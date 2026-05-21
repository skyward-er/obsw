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

#include <Main/Configs/ExternalPinConfig.h>
#include <Main/Configs/PinHandlerConfig.h>
#include <Main/PinHandler/PinData.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>

using namespace std::chrono;
using namespace Main;
using namespace Boardcore;
using namespace Common;
using namespace miosix;
using namespace Config::ExternalPin;

bool PinHandler::isStarted() { return started; }

bool PinHandler::start()
{
    expander = &getModule<GpioExpander>()->getExpander();

    TaskScheduler& scheduler =
        getModule<BoardScheduler>()->getPinObserverScheduler();

    pinObserver = std::make_unique<PinObserver>(
        scheduler, milliseconds{Config::PinHandler::POLL_INTERVAL}.count());

    externalPinObserver = std::make_unique<ExternalPinObserver>(
        scheduler, expander,
        milliseconds{Config::PinHandler::POLL_INTERVAL}.count());

#define CHECK_RESULT(x)                                       \
    if (!x)                                                   \
    {                                                         \
        LOG_ERR(logger, "Failed to register pin callback\n"); \
        return false;                                         \
    }

    CHECK_RESULT(pinObserver->registerPinCallback(
        sense::detachRamp::getPin(),
        [this](PinTransition transition, auto pinData)
        { onRampPinTransition(transition, pinData); },
        Config::PinHandler::RAMP_PIN_THRESHOLD))

    CHECK_RESULT(externalPinObserver->registerPinCallback(
        DETACH_NOSECONE, [this](PinTransition transition, auto pinData)
        { onDetachPayloadTransition(transition, pinData); },
        Config::PinHandler::RAMP_PIN_THRESHOLD))

    CHECK_RESULT(externalPinObserver->registerPinCallback(
        EXPULSION_SENSE, [this](PinTransition transition, auto pinData)
        { onExpulsionSenseTransition(transition, pinData); },
        Config::PinHandler::RAMP_PIN_THRESHOLD))

    CHECK_RESULT(externalPinObserver->registerPinCallback(
        RELEASER_SENSE, [this](PinTransition transition, auto pinData)
        { onReleaserSenseTransition(transition, pinData); },
        Config::PinHandler::RAMP_PIN_THRESHOLD))

    started = true;
    return true;
}

PinData PinHandler::getPinData(PinList pin)
{
    switch (pin)
    {
        case PinList::RAMP_PIN:
            return pinObserver->getPinData(sense::detachRamp::getPin());
        case PinList::DETACH_NOSECONE_PIN:
            return externalPinObserver->getPinData(DETACH_NOSECONE);
        case PinList::EXPULSION_SENSE:
            return externalPinObserver->getPinData(EXPULSION_SENSE);
        case PinList::RELEASER_SENSE:
            return externalPinObserver->getPinData(RELEASER_SENSE);
        default:
            return PinData{};
    }
}

void PinHandler::logPin(PinList pin, const PinData& data)
{
    sdLogger.log(PinChangeData{
        .timestamp        = TimestampTimer::getTimestamp(),
        .pinId            = static_cast<uint8_t>(pin),
        .lastState        = data.lastState,
        .detectionDelayMs = data.getLastDetectionDelay().count(),
        .lastTransitionTs = duration_cast<microseconds>(
                                data.lastTransitionTs.time_since_epoch())
                                .count(),
        .lastStateChangeTs = duration_cast<microseconds>(
                                 data.lastStateChangeTs.time_since_epoch())
                                 .count(),
        .changesCount = data.changesCount,
    });
}

void PinHandler::onRampPinTransition(PinTransition transition,
                                     const PinData& data)
{
    logPin(PinList::RAMP_PIN, data);
    LOG_INFO(logger, "onRampPinTransition {}", static_cast<int>(transition));

    if (transition == Config::PinHandler::RAMP_PIN_TRIGGER)
    {
        rampPinDetectionDelay = data.getLastDetectionDelay();

        EventBroker::getInstance().post(FLIGHT_LAUNCH_PIN_DETACHED,
                                        TOPIC_FLIGHT);
    }
}

void PinHandler::onDetachPayloadTransition(PinTransition transition,
                                           const PinData& data)
{
    logPin(PinList::DETACH_NOSECONE_PIN, data);
    LOG_INFO(logger, "onDetachPayloadTransition {}",
             static_cast<int>(transition));
}

void PinHandler::onExpulsionSenseTransition(PinTransition transition,
                                            const PinData& data)
{
    logPin(PinList::EXPULSION_SENSE, data);
    LOG_INFO(logger, "onExpulsionSenseTransition {}",
             static_cast<int>(transition));
}

void PinHandler::onReleaserSenseTransition(PinTransition transition,
                                           const PinData& data)
{
    logPin(PinList::RELEASER_SENSE, data);
    LOG_INFO(logger, "onReleaserSenseTransition {}",
             static_cast<int>(transition));
}
