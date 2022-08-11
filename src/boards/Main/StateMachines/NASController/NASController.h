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

#include <Singleton.h>
#include <algorithms/NAS/NAS.h>
#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>

#include "NASControllerData.h"

namespace Main
{

class NASController : public Boardcore::FSM<NASController>,
                      public Boardcore::Singleton<NASController>
{
    friend Boardcore::Singleton<NASController>;

public:
    bool start() override;

    void update();

    void initializeOrientationAndPressure();

    void setCoordinates(Eigen::Vector2f position);

    void setOrientation(float yaw, float pitch, float roll);

    void setReferenceAltitude(float altitude);

    void setReferenceTemperature(float temperature);

    NASControllerStatus getStatus();

    Boardcore::NASState getNasState();

    void setReferenceValues(const Boardcore::ReferenceValues reference);

    Boardcore::ReferenceValues getReferenceValues();

    void state_idle(const Boardcore::Event& event);
    void state_calibrating(const Boardcore::Event& event);
    void state_ready(const Boardcore::Event& event);
    void state_active(const Boardcore::Event& event);
    void state_end(const Boardcore::Event& event);

private:
    NASController();
    ~NASController();

    void logStatus(NASControllerState state);

    NASControllerStatus status;
    Boardcore::NAS nas;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("nas");
};

}  // namespace Main
