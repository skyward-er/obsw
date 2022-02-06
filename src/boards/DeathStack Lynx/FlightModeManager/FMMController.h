/* Copyright (c) 2015-2021 Skyward Experimental Rocketry
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

#pragma once

#include <DeathStack.h>
#include <FlightModeManager/FMMStatus.h>
#include <LoggerService/LoggerService.h>
#include <Singleton.h>
#include <events/Events.h>
#include <events/HSM.h>
#include <miosix.h>

using miosix::FastMutex;
using miosix::Lock;

namespace DeathStackBoard
{

/**
 * @brief Implementation of the Flight Mode Manager Finite State Machine.
 */
class FMMController : public Boardcore::HSM<FMMController>
{
public:
    FMMController();
    ~FMMController();

    Boardcore::State state_initialization(const Boardcore::Event& ev);

    /**
     * @brief Handle TC_BOARD_RESET and TC_FORCE_LIFTOFF (super-state).
     */
    Boardcore::State state_onGround(const Boardcore::Event& ev);

    /**
     * @brief First state, wait for sensors and objects initialization.
     */
    Boardcore::State state_init(const Boardcore::Event& ev);

    /**
     * @brief "Low power" state, log only if requested by TC.
     */
    Boardcore::State state_initDone(const Boardcore::Event& ev);

    /**
     * @brief Init error, get stuck.
     */
    Boardcore::State state_initError(const Boardcore::Event& ev);

    /**
     * @brief Test mode, listen to serial and print stuff on serial.
     */
    Boardcore::State state_testMode(const Boardcore::Event& ev);

    /**
     * @brief Calibrating sensors.
     */
    Boardcore::State state_sensorsCalibration(const Boardcore::Event& ev);

    /**
     * @brief Calibrating ADA and NAS.
     */
    Boardcore::State state_algosCalibration(const Boardcore::Event& ev);

    /**
     * @brief All good, waiting for arm.
     */
    Boardcore::State state_disarmed(const Boardcore::Event& ev);

    /**
     * @brief Ready to launch, listening detachment pin (or command).
     */
    Boardcore::State state_armed(const Boardcore::Event& ev);

    /**
     * @brief Handle TC_OPEN and END_MISSION (super-state).
     */
    Boardcore::State state_flying(const Boardcore::Event& ev);

    /**
     * @brief Liftoff.
     */
    Boardcore::State state_ascending(const Boardcore::Event& ev);

    /**
     * @brief Apogee reached, deploy drogue and wait half altitude (or manual
     * mode).
     */
    Boardcore::State state_drogueDescent(const Boardcore::Event& ev);

    /**
     * @brief Descent super-state.
     */
    Boardcore::State state_terminalDescent(const Boardcore::Event& ev);

    /**
     * @brief Close file descriptors.
     */
    Boardcore::State state_landed(const Boardcore::Event& ev);

    FMMStatus getStatus() { return status; }

private:
    void logState(FMMState current_state);

    LoggerService& logger;

    FMMStatus status;

    uint16_t end_mission_d_event_id = 0;

    bool ada_ready = false;
    bool nas_ready = false;

    Boardcore::PrintLogger printLogger =
        Boardcore::Logging::getLogger("deathstack.fsm.fmm");
};

}  // namespace DeathStackBoard
