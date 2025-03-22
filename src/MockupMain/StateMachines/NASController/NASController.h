/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Niccolò Betto, Davide Basso
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

#include <algorithms/NAS/NAS.h>
#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>
#include <utils/DependencyManager/DependencyManager.h>

#include "NASControllerData.h"

namespace MockupMain
{
class BoardScheduler;
class Sensors;
class FlightStatsRecorder;

class NASController
    : public Boardcore::FSM<NASController>,
      public Boardcore::InjectableWithDeps<BoardScheduler, Sensors,
                                           FlightStatsRecorder>
{
public:
    /**
     * @brief Initializes the NAS controller.
     *
     * Sets the initial FSM state to idle, subscribes to NAS and flight events
     * and initializes the NAS algorithm.
     */
    NASController();

    /**
     * @brief Adds the NAS update function into the scheduler and starts the FSM
     * thread
     */
    bool start() override;

    Boardcore::NASState getNasState();
    Boardcore::ReferenceValues getReferenceValues();

    NASControllerState getState();

    void setOrientation(const Eigen::Quaternionf& orientation);

    void setReferenceAltitude(Boardcore::Units::Length::Meter altitude);
    void setReferenceTemperature(float temperature);
    void setReferenceCoordinates(float latitude, float longitude);

private:
    void calibrate();

    /**
     * @brief Update the NAS estimation
     */
    void update();

    // FSM states
    void Init(const Boardcore::Event& event);
    void Calibrating(const Boardcore::Event& event);
    void Ready(const Boardcore::Event& event);
    void Active(const Boardcore::Event& event);
    void End(const Boardcore::Event& event);

    void updateState(NASControllerState newState);

    std::atomic<NASControllerState> state{NASControllerState::INIT};

    Boardcore::NAS nas;  ///< The NAS algorithm instance
    miosix::FastMutex nasMutex;

    int magDecimateCount  = 0;
    int acc1gSamplesCount = 0;
    bool acc1g            = false;

    uint64_t lastGyroTimestamp = 0;
    uint64_t lastAccTimestamp  = 0;
    uint64_t lastMagTimestamp  = 0;
    uint64_t lastGpsTimestamp  = 0;
    uint64_t lastBaroTimestamp = 0;

    std::atomic<bool> started{false};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("NAS");
};

}  // namespace MockupMain
