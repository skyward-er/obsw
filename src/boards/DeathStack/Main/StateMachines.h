/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <LoggerService/LoggerService.h>
#include <NavigationAttitudeSystem/NASData.h>
#include <drivers/gps/ublox/UbloxGPS.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/MS580301BA07/MS580301BA07.h>

#ifdef HARDWARE_IN_THE_LOOP
#include <hardware_in_the_loop/HIL_sensors/HILSensors.h>
#endif

namespace DeathStackBoard
{

class FMMController;
class DeploymentController;

template <typename Press, typename GPS>
class ADAController;

template <typename IMU, typename Press, typename GPS>
class NASController;

template <typename T>
class AirBrakesController;

class StateMachines
{
public:
#ifdef HARDWARE_IN_THE_LOOP
    using IMUType     = HILImu;
    using IMUDataType = HILImuData;

    using PressType     = HILBarometer;
    using PressDataType = HILBaroData;

    using GPSType     = HILGps;
    using GPSDataType = HILGpsData;
#else
    using IMUType     = BMX160;
    using IMUDataType = BMX160Data;

    using PressType     = MS580301BA07;
    using PressDataType = MS5803Data;

    using GPSType     = UbloxGPS;
    using GPSDataType = UbloxGPSData;
#endif

    using ADAControllerType = ADAController<PressDataType, GPSDataType>;
    using NASControllerType =
        NASController<IMUDataType, PressDataType, GPSDataType>;
    using AirBrakesControllerType = AirBrakesController<NASData>;

    DeploymentController* dpl_controller;
    FMMController* fmm;
    ADAControllerType* ada_controller;
    NASControllerType* nas_controller;
    AirBrakesControllerType* arb_controller;

    StateMachines(IMUType& imu, PressType& press, GPSType& gps,
                  TaskScheduler* scheduler);

    ~StateMachines();

    bool start();

    void setReferenceTemperature(float t);

    void setInitialOrientation(float roll, float pitch, float yaw);

    void setInitialCoordinates(float latitude, float longitude);

private:
    void addAlgorithmsToScheduler(TaskScheduler* scheduler);

    LoggerService& logger = *(LoggerService::getInstance());
};

}  // namespace DeathStackBoard