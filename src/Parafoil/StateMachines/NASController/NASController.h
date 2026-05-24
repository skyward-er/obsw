/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <Parafoil/AlgoReference/AlgoReference.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/ADAController/ADAController.h>
#include <algorithms/ANAS/ANAS0.h>
#include <algorithms/ANAS/ANAS0_types.h>
#include <algorithms/ANAS/ANASData.h>
#include <algorithms/NASDAQ/NASDAQ0.h>
#include <algorithms/NASDAQ/NASDAQ0_types.h>
#include <algorithms/NASDAQ/NASDAQData.h>
#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>
#include <utils/DependencyManager/DependencyManager.h>

#include "NASControllerData.h"

namespace Parafoil
{
using namespace Boardcore::Units::Length;
class BoardScheduler;
class Sensors;
class FlightStatsRecorder;
class ADAController;

class NASController
    : public Boardcore::FSM<NASController>,
      public Boardcore::InjectableWithDeps<BoardScheduler, Sensors,
                                           FlightStatsRecorder, AlgoReference,
                                           ADAController>,
      public ReferenceSubscriber
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
    [[nodiscard]] bool start() override;

    Boardcore::ANASState NASController::getANASState();
    Boardcore::NASDAQState NASController::getNASDAQState();
    Boardcore::ReferenceValues getReferenceValues();

    NASControllerState getState();

    void setOrientation(const Eigen::Quaternionf& orientation);

    void setReferenceAltitude(float altitude);
    void setReferenceTemperature(float temperature);
    void setReferenceCoordinates(float latitude, float longitude);

    void onReferenceChanged(const Boardcore::ReferenceValues& ref) override;
    void NASController::onANASReferenceChanged();
    void NASController::onNASDAQReferenceChanged();

    Meter getAltitude();

private:
    void calibrate();
    void initNASDAQ();
    void initANAS();

    void updateANAS();
    void updateNASDAQ();

    /**
     * @brief Update the NAS estimation
     */
    void update();

    // FSM states
    void Init(const Boardcore::Event& event);
    void Calibrating(const Boardcore::Event& event);
    void Ready(const Boardcore::Event& event);
    void Active(const Boardcore::Event& event);
    void Descent(const Boardcore::Event& event);
    void End(const Boardcore::Event& event);

    void updateState(NASControllerState newState);

    std::atomic<NASControllerState> state{NASControllerState::INIT};

    miosix::FastMutex nasMutex;
    NASDAQ0 nasdaq;  ///< The NASDAQ algorithm instance
    ANAS0 anas;

    std::list<Meter> altitudeSamples;

    std::atomic<bool> started{false};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("NAS");

    size_t anasID;    //< ANAS task id
    size_t nasdaqID;  //< NASDAQ task id

    ReferenceValues reference;
};

}  // namespace Parafoil
