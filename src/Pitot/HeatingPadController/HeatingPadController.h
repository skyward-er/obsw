/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Leonardo Montecchi
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

#include <Pitot/Buses.h>
#include <Pitot/Sensors/SensorData.h>
#include <utils/DependencyManager/DependencyManager.h>
#include <algorithms/SchmittTrigger/SchmittTrigger.h>

#include "HeatingPadConfigData.h"

namespace Pitot
{
    
class BoardScheduler;
class Sensors;

class HeatingPadController : public Boardcore::InjectableWithDeps<BoardScheduler, Sensors>
{
    public:
        explicit HeatingPadController(HeatingPadConfig config);

        HeatingPadController() {}

        bool start();
        bool isStarted();

        void enable();
        void disable();
        bool isEnabled();

        void setTargetTemperature(float temperature);

        bool heatingPadSense();

        void enableHeatingPad();
        void disableHeatingPad();

        void update();

    private:

        Boardcore::Units::Frequency::Hertz updateRate{0};

        Boardcore::SchmittTrigger schmittTrigger{0, 0};

        Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
        Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("HeatingPadController");

        std::atomic<bool> started{false};
        std::atomic<bool> running{false};
};

}  // namespace Pitot
