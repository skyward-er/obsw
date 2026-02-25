/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus
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

#include <RIGv3/Actuators/Actuators.h>
#include <RIGv3/BoardScheduler.h>
#include <RIGv3/Registry/Registry.h>
#include <RIGv3/Sensors/Sensors.h>
#include <diagnostic/PrintLogger.h>
#include <events/HSM.h>
#include <logger/Logger.h>

#include <chrono>

#include "FiringSequenceData.h"

namespace RIGv3
{
class FiringSequenceHSM
    : public Boardcore::InjectableWithDeps<Sensors, Actuators, BoardScheduler,
                                           Registry>,
      public Boardcore::HSM<FiringSequenceHSM>
{
public:
    FiringSequenceHSM();

    void setFiringParams(uint32_t fullThrottleTime, uint32_t lowThrottleTime,
                         float pilotFlameOxPosition,
                         float pilotFlameFuelPosition,
                         float lowThrottleOxPosition,
                         float lowThrottleFuelPosition);

    void setPressureThresholds(float igniterThreshold,
                               float pilotFlameThreshold);

    FiringSequenceState getState();

    bool start();

private:
    void checkIgniterPressure();
    void checkPilotFlamePressure();

    Boardcore::State state_ready(const Boardcore::Event& event);
    Boardcore::State state_igniter(const Boardcore::Event& event);
    Boardcore::State state_igniter_wait(const Boardcore::Event& event);
    Boardcore::State state_pilot_flame(const Boardcore::Event& event);
    Boardcore::State state_pilot_flame_wait(const Boardcore::Event& event);
    Boardcore::State state_ramp_up(const Boardcore::Event& event);
    Boardcore::State state_full_throttle(const Boardcore::Event& event);
    Boardcore::State state_low_throttle(const Boardcore::Event& event);
    Boardcore::State state_ended(const Boardcore::Event& event);

    void updateAndLogStatus(FiringSequenceState state);

    Boardcore::Logger& sdLogger = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("firing_sequence");

    std::atomic<FiringSequenceState> state{FiringSequenceState::IDLE};

    uint16_t nextEventId = -1;

    uint8_t igniterFlameSamples = 0;
    uint8_t pilotFlameSamples   = 0;

    float igniterPressureThreshold    = 0.0f;
    float pilotFlamePressureThreshold = 0.0f;

    bool paramsSet = false;
};
}  // namespace RIGv3
