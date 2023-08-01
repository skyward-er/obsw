/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include "Sensors.h"

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace std;
using namespace miosix;
using namespace Boardcore;

constexpr int SAMPLE_PERIOD_VN300 = 20;

namespace Antennas
{
Sensors::Sensors() {}

bool Sensors::start()
{
    if (!vn300Init())
    {
        return false;
    }

    sm = new SensorManager(sensorsMap);
    if (!sm->start())
    {
        LOG_ERR(logger, "Sensor Manager failed to start");
        return false;
    }

    return true;
}

bool Sensors::vn300Init()
{
    vn300 = new Boardcore::VN300(
        ModuleManager::getInstance().get<Buses>()->usart2, 115200);

    SensorInfo info("VN300", SAMPLE_PERIOD_VN300,
                    bind(&Sensors::vn300Callback, this));

    sensorsMap.emplace(make_pair(vn300, info));
    return true;
}

void Sensors::vn300Callback()
{
    Logger::getInstance().log(vn300->getLastSample());
    Logger::getInstance().log(vn300->getLastError());
}

VN300Data Sensors::getVN300LastSample() { return vn300->getLastSample(); }

}  // namespace Antennas