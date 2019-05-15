/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#ifndef SRC_SHARED_BOARDS_HELITEST_ROGALLINA_DESERIALIZE_H
#define SRC_SHARED_BOARDS_HELITEST_ROGALLINA_DESERIALIZE_H

#include <fstream>
#include <iostream>

#include "DeathStack/SensorManager/SensorManagerData.h"
#include "DeathStack/SensorManager/Sensors/AD7994WrapperData.h"
#include "DeathStack/SensorManager/Sensors/ADCWrapperData.h"
#include "DeathStack/SensorManager/Sensors/PiksiData.h"
#include "logger/Deserializer.h"
#include "logger/LogStats.h"
#include "sensors/MPU9250/MPU9250Data.h"
#include "scheduler/TaskSchedulerData.h"

using std::ofstream;

using namespace DeathStackBoard;

template <typename T>
void print(T& t, ostream& os)
{
    t.print(os);
}

void registerTypes(Deserializer& ds)
{
    // LogStats
    ds.registerType<LogStats>(print<LogStats>, LogStats::header());
    ds.registerType<SensorManagerStatus>(print<SensorManagerStatus>,
                                         SensorManagerStatus::header());
    ds.registerType<LM75BData>(print<LM75BData>, LM75BData::header());
    ds.registerType<SensorStatus>(print<SensorStatus>, SensorStatus::header());

    ds.registerType<AD7994WrapperData>(print<AD7994WrapperData>,
                                       AD7994WrapperData::header());

    ds.registerType<BatteryVoltageData>(print<BatteryVoltageData>,
                                        BatteryVoltageData::header());
    ds.registerType<CurrentSenseData>(print<CurrentSenseData>,
                                      CurrentSenseData::header());

    ds.registerType<MPU9250Data>(print<MPU9250Data>, MPU9250Data::header());

    ds.registerType<PiksiData>(print<PiksiData>, PiksiData::header());
    ds.registerType<TaskStatResult>(print<TaskStatResult>, TaskStatResult::header());

    
}

#endif
