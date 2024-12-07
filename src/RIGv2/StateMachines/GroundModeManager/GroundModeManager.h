/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <RIGv2/Actuators/Actuators.h>
#include <RIGv2/CanHandler/CanHandler.h>
#include <RIGv2/Configs/GMMConfig.h>
#include <RIGv2/Registry/Registry.h>
#include <RIGv2/Sensors/Sensors.h>
#include <diagnostic/PrintLogger.h>
#include <events/HSM.h>
#include <logger/Logger.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <atomic>

#include "GroundModeManagerData.h"

namespace RIGv2
{

class GroundModeManager
    : public Boardcore::InjectableWithDeps<Sensors, Actuators, Registry,
                                           CanHandler>,
      public Boardcore::HSM<GroundModeManager>
{
public:
    GroundModeManager();

    GroundModeManagerState getState();

    void setIgnitionTime(uint32_t time);

private:
    Boardcore::State state_idle(const Boardcore::Event& event);
    Boardcore::State state_init(const Boardcore::Event& event);
    Boardcore::State state_init_error(const Boardcore::Event& event);
    Boardcore::State state_disarmed(const Boardcore::Event& event);
    Boardcore::State state_armed(const Boardcore::Event& event);
    Boardcore::State state_firing(const Boardcore::Event& event);
    Boardcore::State state_igniting(const Boardcore::Event& event);
    Boardcore::State state_oxidizer(const Boardcore::Event& event);
    Boardcore::State state_cooling(const Boardcore::Event& event);

    void updateAndLogStatus(GroundModeManagerState state);

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("gmm");

    std::atomic<GroundModeManagerState> state{GroundModeManagerState::IDLE};

    uint16_t openOxidantDelayEventId = -1;
    uint16_t coolingDelayEventId     = -1;
};

}  // namespace RIGv2
