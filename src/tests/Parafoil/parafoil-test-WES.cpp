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
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/WindEstimationScheme/WindEstimation.h>
#include <miosix.h>
#include <utils/Debug.h>

#include <utils/ModuleManager/ModuleManager.hpp>
#include <vector>

using namespace miosix;
using namespace Boardcore;
using namespace Parafoil;
using namespace std;

class SensorsMock : public Sensors
{
public:
    SensorsMock() { testValue = nullptr; }

    bool startModule() override { return true; }

    UBXGPSData getUbxGpsLastSample()
    {
        UBXGPSData data;

        // if out of bounds set fix to 0
        if (index >= testValue->size() - 1)
        {
            return data;
        }

        // else fake data
        data.fix           = 1;
        data.velocityNorth = (*testValue)[index][0];
        data.velocityEast  = (*testValue)[index][1];
        index++;
        return data;
    }

    void setTestValue(vector<vector<float>>* testValue)
    {
        this->testValue = testValue;
    }

private:
    vector<vector<float>>* testValue;
    size_t index = 0;
};

class WindEstimationMock : public WindEstimation
{
public:
    WindEstimationMock() : WindEstimation() {}
};

int main()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Initialize the modules
    Sensors* sensors                = new SensorsMock();
    WindEstimation* wind_estimation = new WindEstimationMock();

    // Insert the modules

    if (!modules.insert<Sensors>(sensors))
    {
        TRACE("Error inserting Sensor\n");
    }
    if (!modules.insert<WindEstimation>(wind_estimation))
    {
        TRACE("Error inserting wind estimation\n");
    }

    // start the scheduler
    if (!BoardScheduler::getInstance().getScheduler().start())
    {
        TRACE("Error starting the General Purpose Scheduler\n");
    }

    // Start the modules
    if (!ModuleManager::getInstance().get<Sensors>()->startModule())
    {
        TRACE("Error starting Sensors\n");
    }

    if (!ModuleManager::getInstance().get<WindEstimation>()->startModule())
    {
        TRACE("Error starting WindEstimation\n");
    }

    WindEstimation& wind_estimation_module = *(modules.get<WindEstimation>());

    vector<vector<float>>* values = new vector<vector<float>>{
        {-100.0000, 78.7071}, {-99.9686, 78.7290}, {-99.9372, 78.7501}};
    TRACE("values size %d\n", (*values).size());
    BoardScheduler::getInstance().getScheduler().start();
    reinterpret_cast<SensorsMock*>(sensors)->setTestValue(values);
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
