/* 
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

#include "events/FSM.h"
#include "DeploymentData.h"
#include "ThermalCutter/Cutter.h"
#include "DeathStack/LogProxy/LogProxy.h"
#include "Motor/MotorDriver.h"

class PinObserver;

namespace DeathStackBoard
{

class DeploymentController : public FSM<DeploymentController>
{
  public:
    DeploymentController(PinObserver* pin_observer);
    ~DeploymentController();

    void state_idle(const Event &ev);

    void state_cuttingDrogue(const Event &ev);
    void state_cuttingMain(const Event &ev);

    void state_openingNosecone(const Event &ev);
    void state_closingNosecone(const Event &ev);

  private:
    /**
     * @brief Logs the DeploymentStatus struct updating the timestamp and the 
     * current state
     * 
     * @param current_state 
     */
    void logStatus(DeploymentCTRLState current_state)
    {
        status.state = current_state;
        status.timestamp = miosix::getTick();
        logger.log(status);
    }
    /**
     * @brief Logs the DeploymentStatus struct updating the timestamp
     */
    void logStatus()
    {
        status.timestamp = miosix::getTick();
        logger.log(status);
    }

    Cutter cutter{};
    DeploymentStatus status;
    MotorDriver motor;
    
    LoggerProxy &logger = *(LoggerProxy::getInstance());

    bool min_open_time_elapsed = false;
    bool nc_detached = false;
    bool cut_main = false;

    uint16_t delayed_ev_id_1 = 0;
    uint16_t delayed_ev_id_2 = 0;
};

} // namespace DeathStackBoard