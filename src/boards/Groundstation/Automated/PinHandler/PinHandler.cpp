/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Nicolò Caruso
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

#include <Groundstation/Automated/Config/PinHandlerConfig.h>
#include <Groundstation/Automated/PinHandler/PinHandler.h>
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

namespace Antennas
{
PinHandler::PinHandler() : scheduler(), pin_observer(scheduler)
{
    pin_observer.registerPinCallback(
        miosix::commBox::switchArm::getPin(),
        bind(&PinHandler::onArmTransition, this, _1),
        PinHandlerConfig::ARM_SWITCH_THRESHOLD);

    pin_observer.registerPinCallback(
        miosix::commBox::switchActive::getPin(),
        bind(&PinHandler::onActiveTransition, this, _1),
        PinHandlerConfig::ACTIVE_SWITCH_THRESHOLD);
}

bool PinHandler::start() { return scheduler.start(); }

bool PinHandler::isStarted() { return scheduler.isRunning(); }

void PinHandler::onArmTransition(PinTransition transition)
{
    if (transition == Boardcore::PinTransition::RISING_EDGE)
    {
        EventBroker::getInstance().post(Common::Events::TMTC_ARP_ARM,
                                        Common::Topics::TOPIC_TMTC);

        // Log the event
        PinData data = getPinData(PinList::ARM_SWITCH);
        PinChangeData log{TimestampTimer::getTimestamp(), PinList::ARM_SWITCH,
                          data.changesCount};
        Logger::getInstance().log(log);
    }
    else
    {
        EventBroker::getInstance().post(Common::Events::TMTC_ARP_DISARM,
                                        Common::Topics::TOPIC_TMTC);

        // Log the event
        PinData data = getPinData(PinList::ARM_SWITCH);
        PinChangeData log{TimestampTimer::getTimestamp(), PinList::ARM_SWITCH,
                          data.changesCount};
        Logger::getInstance().log(log);
    }
}

void PinHandler::onActiveTransition(PinTransition transition)
{
    if (transition == Boardcore::PinTransition::RISING_EDGE)
    {
        EventBroker::getInstance().post(Common::Events::TMTC_ARP_FOLLOW,
                                        Common::Topics::TOPIC_ARP);

        // Log the event
        PinData data = getPinData(PinList::ACTIVE_SWITCH);
        PinChangeData log{TimestampTimer::getTimestamp(),
                          PinList::ACTIVE_SWITCH, data.changesCount};
        Logger::getInstance().log(log);
    }

    else
    {
        EventBroker::getInstance().post(
            Common::Events::TMTC_ARP_RESET_ALGORITHM,
            Common::Topics::TOPIC_ARP);

        // Log the event
        PinData data = getPinData(PinList::ACTIVE_SWITCH);
        PinChangeData log{TimestampTimer::getTimestamp(),
                          PinList::ACTIVE_SWITCH, data.changesCount};
        Logger::getInstance().log(log);
    }
}

PinData PinHandler::getPinData(PinList pin)
{
    switch (pin)
    {
        case PinList::ARM_SWITCH:
        {
            return pin_observer.getPinData(
                miosix::commBox::switchArm::getPin());
        }
        case PinList::ACTIVE_SWITCH:
        {
            return pin_observer.getPinData(
                miosix::commBox::switchActive::getPin());
        }
        default:
        {
            LOG_ERR(logger, "Requested non valid pin");
            return PinData{};
        }
    }
}
}  // namespace Antennas