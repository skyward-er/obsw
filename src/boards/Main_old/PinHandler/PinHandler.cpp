/* Copyright (c) 2019-2023 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio, Matteo Pignataro
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

#include <Main/Configs/PinHandlerConfig.h>
#include <Main/PinHandler/PinHandler.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>

#include <functional>

using namespace Boardcore;
using namespace std;
using namespace std::placeholders;
using namespace Common;

namespace Main
{
PinHandler::PinHandler() : scheduler(), pin_observer(scheduler)
{
    pin_observer.registerPinCallback(
        miosix::gpios::liftoff_detach::getPin(),
        bind(&PinHandler::onLaunchPinTransition, this, _1),
        PinHandlerConfig::LAUNCH_PIN_THRESHOLD);

    pin_observer.registerPinCallback(
        miosix::gpios::nosecone_detach::getPin(),
        bind(&PinHandler::onNoseconeTransition, this, _1),
        PinHandlerConfig::NC_DETACH_PIN_THRESHOLD);

    pin_observer.registerPinCallback(
        miosix::gpios::cut_sense::getPin(),
        bind(&PinHandler::onCutterSenseTransition, this, _1),
        PinHandlerConfig::CUTTER_SENSE_PIN_THRESHOLD);

    pin_observer.registerPinCallback(
        miosix::gpios::exp_sense::getPin(),
        bind(&PinHandler::onExpulsionSenseTransition, this, _1),
        PinHandlerConfig::EXPULSION_PIN_THRESHOLD);
}

bool PinHandler::start() { return scheduler.start(); }

bool PinHandler::isStarted() { return scheduler.isRunning(); }

void PinHandler::onLaunchPinTransition(PinTransition transition)
{
    if (transition == PinHandlerConfig::LAUNCH_PIN_TRIGGER)
    {
        EventBroker::getInstance().post(FLIGHT_LAUNCH_PIN_DETACHED,
                                        TOPIC_FLIGHT);

        // Log the event
        PinData data = getPinData(PinList::LAUNCH_PIN);
        PinChangeData log{TimestampTimer::getTimestamp(), PinList::LAUNCH_PIN,
                          data.changesCount};
        Logger::getInstance().log(log);
    }
}

void PinHandler::onNoseconeTransition(PinTransition transition)
{
    if (transition == PinHandlerConfig::NC_DETACH_PIN_TRIGGER)
    {
        EventBroker::getInstance().post(FLIGHT_NC_DETACHED, TOPIC_FLIGHT);

        // Log the event
        PinData data = getPinData(PinList::NOSECONE_PIN);
        PinChangeData log{TimestampTimer::getTimestamp(), PinList::NOSECONE_PIN,
                          data.changesCount};
        Logger::getInstance().log(log);
    }
}

void PinHandler::onCutterSenseTransition(PinTransition transition)
{
    // Log the event
    PinData data = getPinData(PinList::CUTTER_PRESENCE);
    PinChangeData log{TimestampTimer::getTimestamp(), PinList::CUTTER_PRESENCE,
                      data.changesCount};
    Logger::getInstance().log(log);
}

void PinHandler::onExpulsionSenseTransition(PinTransition transition)
{
    // Log the event
    PinData data = getPinData(PinList::PIN_EXPULSION);
    PinChangeData log{TimestampTimer::getTimestamp(), PinList::PIN_EXPULSION,
                      data.changesCount};
    Logger::getInstance().log(log);
}

PinData PinHandler::getPinData(PinList pin)
{
    switch (pin)
    {
        case PinList::LAUNCH_PIN:
        {
            return pin_observer.getPinData(
                miosix::gpios::liftoff_detach::getPin());
        }
        case PinList::NOSECONE_PIN:
        {
            return pin_observer.getPinData(
                miosix::gpios::nosecone_detach::getPin());
        }
        case PinList::CUTTER_PRESENCE:
        {
            return pin_observer.getPinData(miosix::gpios::cut_sense::getPin());
        }
        case PinList::PIN_EXPULSION:
        {
            return pin_observer.getPinData(miosix::gpios::exp_sense::getPin());
        }
        default:
        {
            LOG_ERR(logger, "Requested non valid pin");
            return PinData{};
        }
    }
}
}  // namespace Main