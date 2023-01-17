/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/WindEstimationScheme/WindEstimation.h>
#include <miosix.h>
#include <utils/Debug.h>

#include <utils/ModuleManager/ModuleManager.hpp>
#include <vector>

using namespace miosix;
using namespace Parafoil;
using namespace std;

int main()
{
    Boardcore::ModuleManager& modules = Boardcore::ModuleManager::getInstance();

    // Initialize the modules
    modules.insert<BoardScheduler>(new BoardScheduler());
    modules.insert<WindEstimation>(new WindEstimation());
    // TODO: probably should add other modules here

    WindEstimation& wind_estimation_module = *(modules.get<WindEstimation>());

    vector<vector<float>>* values = new vector<vector<float>>{
        {-100.0000, 78.7071}, {-99.9686, 78.7290}, {-99.9372, 78.7501}};
    TRACE("values size %d\n", (*values).size());
    modules.get<BoardScheduler>()->getScheduler().start();
    wind_estimation_module.setTestValue(values);
    wind_estimation_module.startWindEstimationSchemeCalibration();
    Thread::sleep(2500);
    wind_estimation_module.stopWindEstimationSchemeCalibration();
    TRACE("Calibration result: n= %f, e= %f \n\n\n\n",
          wind_estimation_module.getWindEstimationScheme()(0),
          wind_estimation_module.getWindEstimationScheme()(1));
    wind_estimation_module.startWindEstimationScheme();
    while (wind_estimation_module.getStatus())
    {
        Thread::sleep(1000);
    }
    TRACE("test ended wes result: n= %f, e= %f \n\n\n\n",
          wind_estimation_module.getWindEstimationScheme()(0),
          wind_estimation_module.getWindEstimationScheme()(1));
    while (1)
    {
        Thread::sleep(1000);
    }
    return 0;
}
