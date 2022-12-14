/* Copyright (c) 2019-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio
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

#include <Main/Configs/PinObserverConfig.h>
#include <common/events/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

#include <functional>

using namespace std;
using namespace std::placeholders;
using namespace miosix;
using namespace Boardcore;
using namespace Common;

namespace Main
{

void PinHandler::onLaunchPinTransition(PinTransition transition)
{
    if (transition == LAUNCH_PIN_TRIGGER)
    {
        Logger::getInstance().log(LiftoffEvent{TimestampTimer::getTimestamp()});
        EventBroker::getInstance().post(Event{FLIGHT_LAUNCH_PIN_DETACHED},
                                        TOPIC_FLIGHT);
    }
}

void PinHandler::onExpulsionPinTransition(PinTransition transition)
{
    if (transition == NC_DETACH_PIN_TRIGGER)
    {
        Logger::getInstance().log(
            ExpulsionEvent{TimestampTimer::getTimestamp()});
        EventBroker::getInstance().post(Event{FLIGHT_NC_DETACHED},
                                        TOPIC_FLIGHT);
    }
}

void PinHandler::onNoseconePinTransition(PinTransition transition)
{
    if (transition == DPL_SERVO_PIN_TRIGGER)
    {
        Logger::getInstance().log(
            NoseconeEvent{TimestampTimer::getTimestamp()});
        EventBroker::getInstance().post(Event{DPL_SERVO_ACTUATION_DETECTED},
                                        TOPIC_DPL);
    }
}

std::map<PinsList, PinData> PinHandler::getPinsData()
{
    std::map<PinsList, PinData> data;

    data[PinsList::LAUNCH_PIN] = PinObserver::getInstance().getPinData(
        sensors::launchpad_detach::getPin());
    data[PinsList::DEPLOYMENT_PIN] =
        PinObserver::getInstance().getPinData(expulsion::sense::getPin());
    data[PinsList::NOSECONE_PIN] = PinObserver::getInstance().getPinData(
        expulsion::nosecone_detach::getPin());

    return data;
}

PinHandler::PinHandler()
{
    PinObserver::getInstance().registerPinCallback(
        expulsion::sense::getPin(),
        bind(&PinHandler::onExpulsionPinTransition, this, _1),
        DPL_SERVO_PIN_THRESHOLD, true);

    PinObserver::getInstance().registerPinCallback(
        sensors::launchpad_detach::getPin(),
        bind(&PinHandler::onLaunchPinTransition, this, _1),
        LAUNCH_PIN_THRESHOLD);

    PinObserver::getInstance().registerPinCallback(
        expulsion::nosecone_detach::getPin(),
        bind(&PinHandler::onNoseconePinTransition, this, _1),
        NC_DETACH_PIN_THRESHOLD, true);
}

}  // namespace Main
