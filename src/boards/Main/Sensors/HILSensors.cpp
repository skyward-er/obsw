/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano
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

// CONTINUE FROM HERE
// ADS131M04Data Sensors::getADS131M04LastSample()
// {
//     PauseKernelLock lock;
//     return ads131m04->getLastSample();
// }

MPXH6115AData Sensors::getStaticPressureLastSample()
{
    PauseKernelLock lock;
    auto baroData = state.barometer->getLastSample();

    MPXH6115AData data;
    data.pressureTimestamp = baroData.pressureTimestamp;
    data.pressure          = baroData.pressure;

    return data;
}

// MPXH6400AData Sensors::getDplPressureLastSample()
// {
//     PauseKernelLock lock;
//     return dplPressure->getLastSample();
// }

// AnalogLoadCellData Sensors::getLoadCellLastSample()
// {
//     PauseKernelLock lock;
//     return loadCell->getLastSample();
// }

// BatteryVoltageSensorData Sensors::getBatteryVoltageLastSample()
// {
//     PauseKernelLock lock;
//     return batteryVoltage->getLastSample();
// }

// InternalADCData Sensors::getInternalADCLastSample()
// {
//     PauseKernelLock lock;
//     return internalAdc->getLastSample();
// }

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
    state.barometer     = new HILBarometer(N_DATA_BARO);
    state.gps           = new HILGps(N_DATA_GPS);
    state.gyro          = new HILGyroscope(N_DATA_GYRO);
    state.magnetometer  = new HILMagnetometer(N_DATA_MAGN);
    state.imu           = new HILImu(N_DATA_IMU);
    state.temperature   = new HILTemperature(N_DATA_TEMP);
    state.kalman        = new HILKalman(N_DATA_KALM);

    sensorsMap = {{state.accelerometer, accelConfig},
                  {state.barometer, baroConfig},
                  {state.magnetometer, magnConfig},
                  {state.imu, imuConfig},
                  {state.gps, gpsConfig},
                  {state.gyro, gyroConfig},
                  {state.temperature, tempConfig},
                  {state.kalman, kalmConfig}};

    // instantiate the sensor manager with the given scheduler
    sensorManager = new SensorManager(
        sensorsMap, &Main::BoardScheduler::getInstance().getScheduler());

    printf("Sensors init ok\n");
    EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);
}

Sensors::~Sensors()
{
    delete state.accelerometer;
    delete state.barometer;
    delete state.gps;
    delete state.gyro;
    delete state.magnetometer;
    delete state.imu;
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
