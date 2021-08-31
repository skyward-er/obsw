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

#include <drivers/adc/ADS1118/ADS1118Data.h>
#include <drivers/gps/ublox/UbloxGPSData.h>
#include <sensors/BMX160/BMX160Data.h>
#include <sensors/BMX160/BMX160WithCorrectionData.h>
#include <sensors/LIS3MDL/LIS3MDLData.h>
#include <sensors/MS580301BA07/MS580301BA07Data.h>
#include <sensors/analog/battery/BatteryVoltageSensorData.h>
#include <sensors/analog/current/CurrentSensorData.h>
#include <sensors/analog/pressure/MPXHZ6130A/MPXHZ6130AData.h>
#include <sensors/analog/pressure/honeywell/SSCDANN030PAAData.h>
#include <sensors/analog/pressure/honeywell/SSCDRRN015PDAData.h>

#include <fstream>
#include <iostream>

#include "ApogeeDetectionAlgorithm/ADAData.h"
#include "AirBrakes/AirBrakesData.h"
//#include "AirBrakes/WindData.h"
#include "DeathStackStatus.h"
#include "Deployment/DeploymentData.h"
#include "FlightModeManager/FMMStatus.h"
#include "LogStats.h"
#include "Main/SensorsData.h"
#include "NavigationAttitudeSystem/NASData.h"
#include "PinHandler/PinHandlerData.h"
#include "System/SystemData.h"
#include "diagnostic/PrintLoggerData.h"
#include "diagnostic/StackData.h"
#include "drivers/Xbee/APIFramesLog.h"
#include "drivers/Xbee/XbeeStatus.h"
#include "drivers/mavlink/MavlinkStatus.h"
#include "events/EventData.h"
#include "logger/Deserializer.h"
#include "scheduler/TaskSchedulerData.h"

// Serialized classes
using std::ofstream;

using namespace DeathStackBoard;

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
    registerType<TaskStatResult>(ds);
    registerType<StackData>(ds);
    registerType<LoggingString>(ds);
    registerType<SystemData>(ds);

    // Sensors
    registerType<CurrentSensorData>(ds);
    registerType<BatteryVoltageSensorData>(ds);
    registerType<UbloxGPSData>(ds);
    registerType<BMX160Data>(ds);
    registerType<BMX160WithCorrectionData>(ds);
    registerType<BMX160GyroscopeCalibrationBiases>(ds);
    registerType<BMX160Temperature>(ds);
    registerType<BMX160FifoStats>(ds);
    registerType<MS5803Data>(ds);
    registerType<MPXHZ6130AData>(ds);
    registerType<SSCDRRN015PDAData>(ds);
    registerType<SSCDANN030PAAData>(ds);
    registerType<LIS3MDLData>(ds);
    registerType<ADS1118Data>(ds);
    registerType<AirSpeedPitot>(ds);

    // Statuses
    registerType<AirBrakesControllerStatus>(ds);
    registerType<DeathStackStatus>(ds);
    registerType<SensorsStatus>(ds);
    registerType<FMMStatus>(ds);
    registerType<PinStatus>(ds);
    registerType<LogStats>(ds);
    registerType<DeploymentStatus>(ds);
    registerType<ADAControllerStatus>(ds);

    // XBee
    registerType<Xbee::APIFrameLog>(ds);
    registerType<Xbee::ATCommandFrameLog>(ds);
    registerType<Xbee::ATCommandResponseFrameLog>(ds);
    registerType<Xbee::ModemStatusFrameLog>(ds);
    registerType<Xbee::TXRequestFrameLog>(ds);
    registerType<Xbee::TXStatusFrameLog>(ds);
    registerType<Xbee::RXPacketFrameLog>(ds);
    registerType<Xbee::XbeeStatus>(ds);
    registerType<MavlinkStatus>(ds);

    // ADA
    registerType<ADAKalmanState>(ds);
    registerType<ADAData>(ds);
    registerType<ADAReferenceValues>(ds);
    registerType<TargetDeploymentAltitude>(ds);

    // NAS
    registerType<NASStatus>(ds);
    registerType<NASKalmanState>(ds);
    registerType<NASReferenceValues>(ds);
    registerType<NASTriadResult>(ds);
    registerType<NASData>(ds);

    // Airbrakes
    registerType<AirBrakesData>(ds);

    // Others
    registerType<EventData>(ds);
    //registerType<WindData>(ds);
}
