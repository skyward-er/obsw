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
#include <diagnostic/PrintLogger.h>
#include <events/FSM.h>

#include "DeploymentData.h"

namespace Main
{

class Deployment : public Boardcore::FSM<Deployment>,
                   public Boardcore::Singleton<Deployment>
{
    friend Boardcore::Singleton<Deployment>;

public:
    DeploymentStatus getStatus();

    void state_init(const Boardcore::Event& event);
    void state_idle(const Boardcore::Event& event);
    void state_nosecone_ejection(const Boardcore::Event& event);
    void state_cutting(const Boardcore::Event& event);

private:
    Deployment();
    ~Deployment();

    DeploymentStatus status;

    void logStatus(DeploymentState state);

    void wiggleServo();
    void startCutting();
    void stopCutting();

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("main.dpl");
};

}  // namespace Main
