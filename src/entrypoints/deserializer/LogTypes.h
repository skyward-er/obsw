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

#include "DeathStack/ADA/ADAStatus.h"
#include "DeathStack//System/SystemData.h"
#include "DeathStack/DeathStackStatus.h"
#include "DeathStack/DeploymentController/DeploymentData.h"
#include "DeathStack/FlightModeManager/FMMStatus.h"
#include "DeathStack/LogProxy/FlightStatsData.h"
#include "DeathStack/PinObserver/PinObserverData.h"
#include "DeathStack/SensorManager/SensorManagerData.h"
#include "DeathStack/SensorManager/Sensors/AD7994WrapperData.h"
#include "DeathStack/SensorManager/Sensors/ADCWrapperData.h"
#include "DeathStack/SensorManager/Sensors/PiksiData.h"
#include "DeathStack/System/EventLog.h"
#include "drivers/mavlink/MavStatus.h"
#include "logger/Deserializer.h"
#include "logger/LogStats.h"
#include "scheduler/TaskSchedulerData.h"
#include "sensors/MPU9250/MPU9250Data.h"
#include <diagnostic/StackData.h>

using std::ofstream;

using namespace DeathStackBoard;

template <typename T>
void print(T& t, ostream& os)
{
    t.print(os);
}

void registerTypes(Deserializer& ds)
{
    ds.registerType<LogStats>(print<LogStats>, LogStats::header());
    ds.registerType<SensorManagerStatus>(print<SensorManagerStatus>,
                                         SensorManagerStatus::header());
    ds.registerType<LM75BData>(print<LM75BData>, LM75BData::header());
    // ds.registerType<SensorStatus>(print<SensorStatus>, SensorStatus::header());

    ds.registerType<AD7994WrapperData>(print<AD7994WrapperData>,
                                       AD7994WrapperData::header());

    ds.registerType<BatteryVoltageData>(print<BatteryVoltageData>,
                                        BatteryVoltageData::header());
    ds.registerType<CurrentSenseData>(print<CurrentSenseData>,
                                      CurrentSenseData::header());

    ds.registerType<MPU9250Data>(print<MPU9250Data>, MPU9250Data::header());

    ds.registerType<PiksiData>(print<PiksiData>, PiksiData::header());
    ds.registerType<TaskStatResult>(print<TaskStatResult>,
                                    TaskStatResult::header());

    ds.registerType<SystemData>(print<SystemData>, SystemData::header());

    ds.registerType<ApogeeDetected>(print<ApogeeDetected>,
                                    ApogeeDetected::header());
    ds.registerType<DplAltitudeReached>(print<DplAltitudeReached>,
                                        DplAltitudeReached::header());
    ds.registerType<ADAStatus>(print<ADAStatus>, ADAStatus::header());
    ds.registerType<KalmanState>(print<KalmanState>, KalmanState::header());
    ds.registerType<KalmanAltitude>(print<KalmanAltitude>,
                                    KalmanAltitude::header());
    ds.registerType<TargetDeploymentAltitude>(
        print<TargetDeploymentAltitude>, TargetDeploymentAltitude::header());
    ds.registerType<ADACalibrationData>(print<ADACalibrationData>,
                                        ADACalibrationData::header());

    ds.registerType<DeploymentStatus>(print<DeploymentStatus>,
                                      DeploymentStatus::header());

    ds.registerType<FMMStatus>(print<FMMStatus>, FMMStatus::header());

    ds.registerType<LiftOffStats>(print<LiftOffStats>, LiftOffStats::header());
    ds.registerType<ApogeeStats>(print<ApogeeStats>, ApogeeStats::header());
    ds.registerType<DrogueDPLStats>(print<DrogueDPLStats>,
                                    DrogueDPLStats::header());
    ds.registerType<MainDPLStats>(print<MainDPLStats>, MainDPLStats::header());

    ds.registerType<PinStatus>(print<PinStatus>, PinStatus::header());

    ds.registerType<MavStatus>(print<MavStatus>, MavStatus::header());

    ds.registerType<EventLog>(print<EventLog>, EventLog::header());
    ds.registerType<DeathStackStatus>(print<DeathStackStatus>,
                                      DeathStackStatus::header());

    // ds.registerType<StackData>(print<StackData>, StackData::header());
    ds.registerType<ReferenceValues>(print<ReferenceValues>, ReferenceValues::header());
    ds.registerType<StackData>(print<StackData>, StackData::header());


}

#endif
