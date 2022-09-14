/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#pragma once

#include <Payload/CanHandler/CanHandler.h>
#include <common/CanConfig.h>
#include <common/events/Events.h>

#include <functional>
#include <map>

namespace Payload
{

namespace CanHandlerConfig
{

// Important to use only TMTC events because the Can sends on that topic
static const std::map<Common::CanConfig::EventId, Common::Events> eventToEvent{
    {Common::CanConfig::EventId::ARM, Common::TMTC_ARM},
    {Common::CanConfig::EventId::DISARM, Common::TMTC_DISARM},
    {Common::CanConfig::EventId::CALIBRATE, Common::TMTC_CALIBRATE},
    {Common::CanConfig::EventId::CAM_ON, Common::TMTC_START_RECORDING},
    {Common::CanConfig::EventId::CAM_OFF, Common::TMTC_STOP_RECORDING},
    {Common::CanConfig::EventId::LIFTOFF, Common::TMTC_FORCE_LAUNCH},
    {Common::CanConfig::EventId::APOGEE, Common::TMTC_FORCE_APOGEE},
};

static const std::map<Common::Events, std::function<void(CanHandler *)>>
    eventToFunction{
        {Common::FLIGHT_ARMED, &CanHandler::sendArmEvent},
        {Common::FLIGHT_DISARMED, &CanHandler::sendDisarmEvent},
        {Common::TMTC_CALIBRATE, &CanHandler::sendCalibrateEvent},
        {Common::TMTC_START_RECORDING, &CanHandler::sendCamOnEvent},
        {Common::TMTC_STOP_RECORDING, &CanHandler::sendCamOffEvent},
    };

}  // namespace CanHandlerConfig

}  // namespace Payload
