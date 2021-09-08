/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <ApogeeDetectionAlgorithm/ADAData.h>
#include <FlightStatsRecorder/FSRData.h>
#include <Main/SensorsData.h>
#include <configs/FlightStatsConfig.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/gps/ublox/UbloxGPSData.h>
#include <events/FSM.h>
#include <sensors/BMX160/BMX160WithCorrectionData.h>
#include <sensors/MS5803/MS5803Data.h>
#include <sensors/analog/current/CurrentSensorData.h>
#include <sensors/analog/pressure/MPXHZ6130A/MPXHZ6130AData.h>
#include <sensors/analog/pressure/honeywell/SSCDANN030PAAData.h>

#ifdef HARDWARE_IN_THE_LOOP
#include <hardware_in_the_loop/HIL_sensors/HILSensors.h>
#endif

namespace DeathStackBoard
{

/**
 * @brief Records statistics about the flight.
 *
 * Statistics includes maximum acceleration during liftoff, maximum altitude,
 * maximum speed etc. In order to do so, we need to know in which stage of the
 * flight we are in, and we do so using a state machine and receiving events
 * from the topic FLIGHT_EVENTS.
 */
class FlightStatsRecorder : public FSM<FlightStatsRecorder>
{
public:
    FlightStatsRecorder();
    ~FlightStatsRecorder();

    void update(const ADAKalmanState& t);
    void update(const ADAData& t);
    void update(const UbloxGPSData& t);
    void update(const BMX160WithCorrectionData& t);
    //void update(const CurrentSensorData& t);
    void update(const MS5803Data& t);      // digitl baro
    void update(const MPXHZ6130AData& t);  // static ports baro
    void update(const SSCDANN030PAAData& t);  // DPL vane baro
    void update(const AirSpeedPitot& t);

#ifdef HARDWARE_IN_THE_LOOP
    void update(const HILImuData& t);
    void update(const HILBaroData& t);
    void update(const HILGpsData& t);
#endif

    /**
     * @brief Wait for liftoff or deployment.
     */
    void state_idle(const Event& ev);

    void state_testingCutters(const Event& ev);

    /**
     * @brief Record stats of the first few seconds of flight.
     */
    void state_liftOff(const Event& ev);

    /**
     * @brief Record stats for the apogee part of the flight.
     */
    void state_ascending(const Event& ev);

    /**
     * @brief Record stats during drogue deployment.
     */
    void state_drogueDeployment(const Event& ev);

    /**
     * @brief Stats during main deployment.
     */
    void state_mainDeployment(const Event& ev);

private:
    LiftOffStats liftoff_stats{};
    ApogeeStats apogee_stats{};
    DrogueDPLStats drogue_dpl_stats{};
    MainDPLStats main_dpl_stats{};
    CutterTestStats cutters_stats{};
    FSRState state      = FSRState::IDLE;
    long long T_liftoff = 0;

    uint16_t ev_timeout_id = 0;

    PrintLogger log = Logging::getLogger("deathstack.fsm.flightstatsrecorder");
};

}  // namespace DeathStackBoard