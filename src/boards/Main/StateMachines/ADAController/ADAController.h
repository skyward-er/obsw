/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#include <Main/Configs/ADAConfig.h>
#include <Singleton.h>
#include <algorithms/ADA/ADA.h>
#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>

#include "ADAControllerData.h"

namespace Main
{

class ADAController : public Boardcore::FSM<ADAController>,
                      public Boardcore::Singleton<ADAController>
{
    friend Boardcore::Singleton<ADAController>;

public:
    bool start() override;

    void update();

    ADAControllerStatus getStatus();

    Boardcore::ADAState getAdaState();

    void setReferenceValues(const Boardcore::ReferenceValues reference);

    Boardcore::ReferenceValues getReferenceValues();

    void state_idle(const Boardcore::Event& event);
    void state_calibrating(const Boardcore::Event& event);
    void state_ready(const Boardcore::Event& event);
    void state_shadow_mode(const Boardcore::Event& event);
    void state_active(const Boardcore::Event& event);
    void state_pressure_stabilization(const Boardcore::Event& event);
    void state_drogue_descent(const Boardcore::Event& event);
    void state_terminal_descent(const Boardcore::Event& event);
    void state_landed(const Boardcore::Event& event);

private:
    ADAController();
    ~ADAController();

    ADAControllerStatus status;

    void logStatus(ADAControllerState state);

    Boardcore::ADA::KalmanFilter::KalmanConfig getADAKalmanConfig();

    void calibrate();

    Boardcore::ADA ada;

    uint16_t detectedApogeeEvents     = 0;
    uint16_t detectedAbkDisableEvents = 0;
    uint16_t detectedDeploymentEvents = 0;
    uint16_t detectedLandingEvents    = 0;

    float deploymentAltitude = ADAConfig::DEFAULT_DEPLOYMENT_ALTITUDE;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("main.ada");
};

}  // namespace Main
