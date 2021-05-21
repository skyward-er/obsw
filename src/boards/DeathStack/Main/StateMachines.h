/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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

#include <sensors/MS580301BA07/MS580301BA07.h>
#include <sensors/BMX160/BMX160.h>
#include <drivers/gps/ublox/UbloxGPS.h>
#include <NavigationSystem/NASData.h>

namespace DeathStackBoard
{

class FlightModeManager;
class DeploymentController;

template <typename Press, typename GPS>
class ADAController;

template <typename IMU, typename Press, typename GPS>
class NASController;

template <typename T>
class AeroBrakesController;

class StateMachines
{
public:
    using ADAControllerType = ADAController<MS5803Data, UbloxGPSData>;
    using NASControllerType = NASController<BMX160Data, MS5803Data, UbloxGPSData>;
    using AeroBrakesControllerType = AeroBrakesController<NASData>;

    DeploymentController* dpl_controller;
    FlightModeManager* fmm;
    ADAControllerType* ada_controller;
    NASControllerType* nas_controller;
    AeroBrakesControllerType* arb_controller;

    StateMachines(BMX160& imu, MS580301BA07& press, UbloxGPS& gps);

    ~StateMachines();

    void start();

private:
    
};

}