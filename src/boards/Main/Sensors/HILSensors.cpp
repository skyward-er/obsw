/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#include "HIL_sensors/HILSensors.h"

#include <Main/Buses.h>
#include <Main/Configs/SensorsConfig.h>
#include <common/events/Events.h>
#include <drivers/interrupt/external_interrupts.h>
#include <events/EventBroker.h>

#include "Main/BoardScheduler.h"
#include "Sensors.h"

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace Common;
using namespace Main::SensorsConfig;

namespace Main
{

bool Sensors::start()
{
    sensorManager->enableAllSensors();
    return sensorManager->start();
}

bool Sensors::isStarted() { return sensorManager->areAllSensorsInitialized(); }

BMX160Data Sensors::getBMX160LastSample()
{
    PauseKernelLock lock;

    auto imuData = state.imu->getLastSample();
    BMX160Data data;

    data.accelerationTimestamp    = imuData.accelerationTimestamp;
    data.accelerationX            = imuData.accelerationX;
    data.accelerationY            = imuData.accelerationY;
    data.accelerationZ            = imuData.accelerationZ;
    data.angularVelocityTimestamp = imuData.angularVelocityTimestamp;
    data.angularVelocityX         = imuData.angularVelocityX;
    data.angularVelocityY         = imuData.angularVelocityY;
    data.angularVelocityZ         = imuData.angularVelocityZ;
    data.magneticFieldTimestamp   = imuData.magneticFieldTimestamp;
    data.magneticFieldX           = imuData.magneticFieldX;
    data.magneticFieldY           = imuData.magneticFieldY;
    data.magneticFieldZ           = imuData.magneticFieldZ;

    return data;
}

BMX160WithCorrectionData Sensors::getBMX160WithCorrectionLastSample()
{
    PauseKernelLock lock;
    auto imuData = state.imu->getLastSample();
    BMX160WithCorrectionData data;

    data.accelerationTimestamp    = imuData.accelerationTimestamp;
    data.accelerationX            = imuData.accelerationX;
    data.accelerationY            = imuData.accelerationY;
    data.accelerationZ            = imuData.accelerationZ;
    data.angularVelocityTimestamp = imuData.angularVelocityTimestamp;
    data.angularVelocityX         = imuData.angularVelocityX;
    data.angularVelocityY         = imuData.angularVelocityY;
    data.angularVelocityZ         = imuData.angularVelocityZ;
    data.magneticFieldTimestamp   = imuData.magneticFieldTimestamp;
    data.magneticFieldX           = imuData.magneticFieldX;
    data.magneticFieldY           = imuData.magneticFieldY;
    data.magneticFieldZ           = imuData.magneticFieldZ;

    return data;
}

MPU9250Data Sensors::getMPU9250LastSample()
{
    PauseKernelLock lock;
    auto imuData  = state.imu->getLastSample();
    auto tempData = state.temperature->getLastSample();
    MPU9250Data data;

    data.accelerationTimestamp    = imuData.accelerationTimestamp;
    data.accelerationX            = imuData.accelerationX;
    data.accelerationY            = imuData.accelerationY;
    data.accelerationZ            = imuData.accelerationZ;
    data.angularVelocityTimestamp = imuData.angularVelocityTimestamp;
    data.angularVelocityX         = imuData.angularVelocityX;
    data.angularVelocityY         = imuData.angularVelocityY;
    data.angularVelocityZ         = imuData.angularVelocityZ;
    data.magneticFieldTimestamp   = imuData.magneticFieldTimestamp;
    data.magneticFieldX           = imuData.magneticFieldX;
    data.magneticFieldY           = imuData.magneticFieldY;
    data.magneticFieldZ           = imuData.magneticFieldZ;
    data.temperatureTimestamp     = tempData.temperatureTimestamp;
    data.temperature              = tempData.temperature;

    return data;
}

MS5803Data Sensors::getMS5803LastSample()
{
    PauseKernelLock lock;
    auto baroData = state.barometer->getLastSample();
    auto tempData = state.temperature->getLastSample();

    return MS5803Data(baroData.pressureTimestamp, baroData.pressure,
                      tempData.temperatureTimestamp, tempData.temperature);
}

UBXGPSData Sensors::getUbxGpsLastSample()
{
    miosix::PauseKernelLock lock;
    auto data = state.gps->getLastSample();
    UBXGPSData ubxData;

    ubxData.gpsTimestamp  = data.gpsTimestamp;
    ubxData.latitude      = data.latitude;
    ubxData.longitude     = data.longitude;
    ubxData.height        = data.height;
    ubxData.velocityNorth = data.velocityNorth;
    ubxData.velocityEast  = data.velocityEast;
    ubxData.velocityDown  = data.velocityDown;
    ubxData.speed         = data.speed;
    ubxData.track         = data.track;
    ubxData.positionDOP   = data.positionDOP;
    ubxData.satellites    = data.satellites;
    ubxData.fix           = data.fix;

    return ubxData;
}

ADS131M04Data Sensors::getADS131M04LastSample()
{
    PauseKernelLock lock;
    return ADS131M04Data{};
}

MPXH6115AData Sensors::getStaticPressureLastSample()
{
    PauseKernelLock lock;
    auto baroData = state.barometer->getLastSample();

    MPXH6115AData data;
    data.pressureTimestamp = baroData.pressureTimestamp;
    data.pressure          = baroData.pressure;

    return data;
}

Boardcore::SSCDRRN015PDAData Sensors::getDifferentialPressureLastSample()
{
    PauseKernelLock lock;
    auto pitotData = state.pitot->getLastSample();

    Boardcore::SSCDRRN015PDAData data;
    data.pressureTimestamp = pitotData.pressureTimestamp;
    data.pressure          = pitotData.pressure;

    return data;
};

MPXH6400AData Sensors::getDplPressureLastSample()
{
    PauseKernelLock lock;
    return MPXH6400AData{};
}

AnalogLoadCellData Sensors::getLoadCellLastSample()
{
    PauseKernelLock lock;
    return AnalogLoadCellData{};
}

BatteryVoltageSensorData Sensors::getBatteryVoltageLastSample()
{
    PauseKernelLock lock;
    return BatteryVoltageSensorData{};
}

InternalADCData Sensors::getInternalADCLastSample()
{
    PauseKernelLock lock;
    return InternalADCData{};
}

void Sensors::calibrate() {}

std::map<string, bool> Sensors::getSensorsState()
{
    std::map<string, bool> sensorsState;

    for (auto sensor : sensorsMap)
        sensorsState[sensor.second.id] =
            sensorManager->getSensorInfo(sensor.first).isInitialized;

    return sensorsState;
}

Sensors::Sensors()
{
    // Definition of the fake sensors for the simulation
    state.accelerometer = new HILAccelerometer(N_DATA_ACCEL);
    state.gyro          = new HILGyroscope(N_DATA_GYRO);
    state.magnetometer  = new HILMagnetometer(N_DATA_MAGN);
    state.imu           = new HILImu(N_DATA_IMU);
    state.barometer     = new HILBarometer(N_DATA_BARO);
    state.pitot         = new HILPitot(N_DATA_PITOT);
    state.gps           = new HILGps(N_DATA_GPS);
    state.temperature   = new HILTemperature(N_DATA_TEMP);
    state.kalman        = new HILKalman(N_DATA_KALM);

    sensorsMap = {{state.accelerometer, accelConfig},
                  {state.gyro, gyroConfig},
                  {state.magnetometer, magnConfig},
                  {state.imu, imuConfig},
                  {state.barometer, baroConfig},
                  {state.pitot, pitotConfig},
                  {state.gps, gpsConfig},
                  {state.temperature, tempConfig},
                  {state.kalman, kalmConfig}};

    // instantiate the sensor manager with the given scheduler
    sensorManager = new SensorManager(
        sensorsMap, &Main::BoardScheduler::getInstance().getScheduler());
}

Sensors::~Sensors()
{
    delete state.accelerometer;
    delete state.gyro;
    delete state.magnetometer;
    delete state.imu;
    delete state.barometer;
    delete state.pitot;
    delete state.gps;
    delete state.temperature;
    delete state.kalman;

    sensorManager->stop();
    delete sensorManager;
}

void Sensors::bmx160Init() {}
void Sensors::bmx160Callback() {}
void Sensors::bmx160WithCorrectionInit() {}
void Sensors::mpu9250Init() {}
void Sensors::ms5803Init() {}
void Sensors::ubxGpsInit() {}
void Sensors::ads131m04Init() {}
void Sensors::staticPressureInit() {}
void Sensors::dplPressureInit() {}
void Sensors::loadCellInit() {}
void Sensors::batteryVoltageInit() {}
void Sensors::internalAdcInit() {}

}  // namespace Main
