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

#include <Main/CanHandler/CanHandler.h>
#include <common/CanConfig.h>
#include <common/events/Events.h>

#include <functional>
#include <map>

namespace Main
{

namespace CanHandlerConfig
{

static const std::map<Common::CanConfig::EventId, Common::Events> eventToEvent{
    {Common::CanConfig::EventId::ARM, Common::TMTC_ARM},
    {Common::CanConfig::EventId::DISARM, Common::TMTC_DISARM},
    {Common::CanConfig::EventId::CAM_ON, Common::TMTC_START_RECORDING},
    {Common::CanConfig::EventId::CAM_OFF, Common::TMTC_STOP_RECORDING},
};

static const std::map<Common::Events, std::function<void(CanHandler *)>>
    eventToFunction{
        {Common::TMTC_ARM, &CanHandler::sendArmEvent},
        {Common::TMTC_DISARM, &CanHandler::sendDisarmEvent},
        {Common::TMTC_START_RECORDING, &CanHandler::sendCamOnEvent},
        {Common::TMTC_STOP_RECORDING, &CanHandler::sendCamOffEvent},
        {Common::FLIGHT_LIFTOFF, &CanHandler::sendLiftoffEvent},
        {Common::FLIGHT_APOGEE_DETECTED, &CanHandler::sendApogeeEvent},
    };

}  // namespace CanHandlerConfig

}  // namespace Main
