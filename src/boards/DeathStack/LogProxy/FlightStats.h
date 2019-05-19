/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <events/FSM.h>

#include "Telemetries.h"

#include "DeathStack/ADA/ADAStatus.h"
#include "DeathStack/SensorManager/Sensors/AD7994WrapperData.h"
#include "DeathStack/configs/FlightStatsConfig.h"
#include "FlightStatsData.h"
#include "DeathStack/SensorManager/Sensors/PiksiData.h"
#include "sensors/MPU9250/MPU9250Data.h"
#include "utils/CircularBuffer.h"

namespace DeathStackBoard
{

class FlightStats : public FSM<FlightStats>
{
public:
    FlightStats();
    ~FlightStats();

    void update(const KalmanState& t);
    void update(const KalmanAltitude& t);
    void update(const AD7994WrapperData& t);
    void update(const MPU9250Data& t);
    void update(const PiksiData& t);

    void state_idle(const Event& ev);
    void state_liftOff(const Event& ev);
    void state_apogee(const Event& ev);
    void state_drogueDeployment(const Event& ev);
    void state_mainDeployment(const Event& ev);

private:
    enum class State
    {
        IDLE,
        LIFTOFF,
        APOGEE,
        DROGUE_DPL,
        MAIN_DPL
    };

    LiftOffStats liftoff_stats{};
    ApogeeStats apogee_stats{};
    DrogueDPLStats drogue_dpl_stats{};
    MainDPLStats main_dpl_stats{};

    State state         = State::IDLE;
    long long T_liftoff = 0;

    uint16_t ev_timeout_id = 0;

    CircularBuffer<Event, DEFERRED_EVENTS_QUEUE_SIZE> deferred_events;
};

}  // namespace DeathStackBoard