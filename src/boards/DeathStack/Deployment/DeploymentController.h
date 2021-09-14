/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <Deployment/DeploymentData.h>
#include <Deployment/DeploymentServo.h>
#include <configs/CutterConfig.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/hbridge/HBridge.h>
#include <drivers/servo/servo.h>
#include <events/Events.h>
#include <events/FSM.h>

using namespace DeathStackBoard::DeploymentConfigs;
using namespace DeathStackBoard::CutterConfig;

namespace DeathStackBoard
{

/**
 * @brief Deployment state machine.
 */
class DeploymentController : public FSM<DeploymentController>
{
public:
    DeploymentController(
        ServoInterface* ejection_servo = new DeploymentServo());
    ~DeploymentController();

    void state_initialization(const Event& ev);
    void state_idle(const Event& ev);
    void state_noseconeEjection(const Event& ev);
    void state_cutting(const Event& ev);

    void ejectNosecone();
    void startCutting();
    void stopCutting();

private:
    DeploymentStatus status;

    ServoInterface* ejection_servo;

    uint16_t ev_nc_open_timeout_id    = 0;
    uint16_t ev_nc_cutting_timeout_id = 0;

    PrintLogger log = Logging::getLogger("deathstack.fsm.dpl");

    void logStatus(DeploymentControllerState current_state);
};

}  // namespace DeathStackBoard
