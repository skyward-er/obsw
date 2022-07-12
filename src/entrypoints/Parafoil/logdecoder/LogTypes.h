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

#include <Parafoil/Wing/WingAlgorithmData.h>
#include <algorithms/NAS/NASState.h>
#include <sensors/BME280/BME280Data.h>
#include <sensors/MPU9250/MPU9250Data.h>
#include <sensors/UBXGPS/UBXGPSData.h>

#include <fstream>
#include <iostream>

//#include "AirBrakes/WindData.h"
//#include "Deployment/DeploymentData.h"
//#include "LogStats.h"
#include <common/SystemData.h>

#include "diagnostic/PrintLoggerData.h"
// #include "diagnostic/StackData.h"
#include "events/EventData.h"
// #include "radio/MavlinkDriver/MavlinkStatus.h"
//#include "logger/Deserializer.h"
#include <Parafoil/ParafoilTestStatus.h>
#include <logger/Deserializer.h>
#include <logger/LoggerStats.h>
#include <scheduler/TaskSchedulerData.h>

using namespace Boardcore;
using namespace Parafoil;

template <typename T>
void print(T& t, ostream& os)
{
    t.print(os);
}

template <typename T>
void registerType(Deserializer& ds)
{
    ds.registerType<T>(print<T>, T::header());
}

void registerTypes(Deserializer& ds)
{
    // Disagnostic
    registerType<TaskStatsResult>(ds);
    registerType<LoggerStats>(ds);
    // registerType<StackData>(ds);
    registerType<LoggingString>(ds);
    registerType<SystemData>(ds);
    registerType<ParafoilTestStatus>(ds);

    // Parafoil data
    registerType<WingAlgorithmData>(ds);

    // Sensors
    registerType<UBXGPSData>(ds);
    registerType<MPU9250Data>(ds);
    registerType<BME280Data>(ds);

    // Nas state
    registerType<NASState>(ds);

    // Mavlink
    // registerType<MavlinkStatus>(ds);

    // Others
    registerType<EventData>(ds);
    // registerType<WindData>(ds);
}
