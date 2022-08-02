/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio, Matteo Pignataro
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
#include <algorithms/NAS/StateInitializer.h>
#include <scheduler/TaskScheduler.h>

#include <Eigen/Core>
#include <functional>

namespace Payload
{

class NASController : public Boardcore::Singleton<NASController>
{
    friend Boardcore::Singleton<NASController>;

public:
    void init();

    bool start();

    void update();

    void initializeOrientationAndPressure();

    void setInitialPosition(Eigen::Vector2f position);

    Boardcore::NASState getNasState();

    void setReferenceValues(const Boardcore::ReferenceValues reference);

    Boardcore::ReferenceValues getReferenceValues();

private:
    NASController();

    Boardcore::NAS nas;

    Eigen::Vector3f initialOrientation;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("NAS");
};

}  // namespace Payload
