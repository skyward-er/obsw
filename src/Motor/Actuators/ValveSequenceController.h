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

#pragma once

#include <Motor/BoardScheduler.h>
#include <common/Events.h>
#include <common/canbus/MotorStatus.h>
#include <events/EventHandler.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Motor
{

class Actuators;
class Sensors;
class CanHandler;

class ValveSequenceController
    : public Boardcore::EventHandler,
      public Boardcore::InjectableWithDeps<Actuators, Sensors, CanHandler,
                                           BoardScheduler>
{
public:
    ValveSequenceController(
        unsigned int stacksize    = miosix::STACK_DEFAULT_FOR_PTHREAD,
        miosix::Priority priority = miosix::MAIN_PRIORITY);

    /**
     * @brief Closes all engine valves in a staged, safe sequence.
     *
     * Closes groups of valves with safety delays in between, in order to avoid
     * power surges.
     */
    void closeValves();

    /**
     * @brief Post-apogee depressurization sequence.
     *
     * Opens OX-VENT, waits for OX-TANK pressure to drop, then opens OX-PRZ,
     * waits for PRZ-TANK pressure to drop, then opens FUEL-PRZ for 30s.
     * Uses hysteresis-based timing for pressure checks with 60s timeouts.
     */
    void depressurizeTanks();

protected:
    void handleEvent(const Boardcore::Event& ev) override;

private:
    enum class DepressurizationState
    {
        IDLE,
        OX_VENTING,
        OX_PRZ_VENTING,
        PRZ_FUEL_VENTING,
        DONE
    };

    void depressurizationUpdate();

    DepressurizationState depressurizationState = DepressurizationState::IDLE;
    size_t depressurizationTaskId{0};

    std::chrono::steady_clock::time_point depressurizationStartTime;
    std::chrono::steady_clock::time_point lastPressureOverTime;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("vsc");
};
}  // namespace Motor
