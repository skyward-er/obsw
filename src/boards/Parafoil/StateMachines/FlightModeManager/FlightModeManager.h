/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Angelo Prete
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

#include <diagnostic/PrintLogger.h>
#include <events/HSM.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "FlightModeManagerData.h"

namespace Parafoil
{
class FlightModeManager : public Boardcore::HSM<FlightModeManager>,
                          public Boardcore::Module
{
public:
    FlightModeManager();
    ~FlightModeManager();

    bool startModule() { return start(); }
    FlightModeManagerStatus getStatus();

    /**
     * @brief Super state for when the parafoil is on ground.
     */
    Boardcore::State state_on_ground(const Boardcore::Event& event);

    /**
     * @brief Super state for when the parafoil is on ground.
     */
    Boardcore::State state_init(const Boardcore::Event& event);

    /**
     * @brief State in which the init has failed
     */
    Boardcore::State state_init_error(const Boardcore::Event& event);

    /**
     * @brief State in which the init is done and a calibration event is
     * thrown
     */
    Boardcore::State state_init_done(const Boardcore::Event& event);

    /**
     * @brief Calibration of all sensors.
     */
    Boardcore::State state_sensors_calibration(const Boardcore::Event& event);

    /**
     * @brief Calibration of all algorithms.
     */
    Boardcore::State state_algos_calibration(const Boardcore::Event& event);

    /**
     * @brief The parafoil will accept specific telecommands otherwise
     * considered risky.
     */
    Boardcore::State state_test_mode(const Boardcore::Event& event);

    /**
     * @brief State in which the parafoil is waiting to be dropped.
     */
    Boardcore::State state_ready(const Boardcore::Event& event);

    /**
     * @brief State in which the parafoil wing is opened and starts guiding
     * itself
     */
    Boardcore::State state_wing_descent(const Boardcore::Event& event);

    /**
     * @brief The parafoil ended the flight and closes the log.
     */
    Boardcore::State state_landed(const Boardcore::Event& event);

private:
    void logStatus(FlightModeManagerState state);

    FlightModeManagerStatus status;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("fmm");
};
}  // namespace Parafoil
