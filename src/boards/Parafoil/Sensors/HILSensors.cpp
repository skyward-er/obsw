/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio, Matteo Pignataro
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

#include "HILSensors.h"

#include <Parafoil/Actuators/Actuators.h>
#include <Parafoil/Buses.h>
#include <Parafoil/Configs/SensorsConfig.h>
#include <common/Events.h>
#include <common/ReferenceConfig.h>
#include <drivers/interrupt/external_interrupts.h>
#include <drivers/usart/USART.h>
#include <events/EventBroker.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "HILConfig.h"

using namespace std;
using namespace Boardcore;
using namespace Common;
using namespace ReferenceConfig;
using namespace Parafoil::SensorsConfig;

namespace Parafoil
{

HILSensors::HILSensors()
    : state{new HILAccelerometer(N_DATA_ACCEL),
            new HILBarometer(N_DATA_BARO),
            new HILGps(N_DATA_GPS),
            new HILGyroscope(N_DATA_GYRO),
            new HILMagnetometer(N_DATA_MAGN),
            new HILTemperature(N_DATA_TEMP),
            new HILImu(N_DATA_IMU)}
{
}

HILSensors::~HILSensors()
{
    delete bmx160;
    delete lis3mdl;
    delete ms5803;
    delete ubxGps;
    delete ads1118;
    delete staticPressure;
    delete dplPressure;
    delete pitotPressure;

    delete state.accelerometer;
    delete state.barometer;
    delete state.gps;
    delete state.gyro;
    delete state.magnetometer;
    delete state.imu;
    delete state.temperature;

    sensorManager->stop();
    delete sensorManager;
}

bool HILSensors::startModule()
{
    sensorsMap = {{state.accelerometer, accelConfig},
                  {state.barometer, baroConfig},
                  {state.magnetometer, magnConfig},
                  {state.imu, imuConfig},
                  {state.gps, gpsConfig},
                  {state.gyro, gyroConfig},
                  {state.temperature, tempConfig}};

    // Create the sensor manager
    sensorManager = new SensorManager(sensorsMap);

    return sensorManager->start();
}

bool HILSensors::isStarted()
{
    return sensorManager->areAllSensorsInitialized();
}

BMX160Data HILSensors::getBMX160LastSample()
{
    miosix::PauseKernelLock lock;
    return bmx160 != nullptr ? bmx160->getLastSample() : BMX160Data{};
}

BMX160WithCorrectionData HILSensors::getBMX160WithCorrectionLastSample()
{
    miosix::PauseKernelLock lock;

    auto imuData = state.imu->getLastSample();
    BMX160WithCorrectionData data;

    data.accelerationTimestamp  = imuData.accelerationTimestamp;
    data.accelerationX          = imuData.accelerationX;
    data.accelerationY          = imuData.accelerationY;
    data.accelerationZ          = imuData.accelerationZ;
    data.angularSpeedTimestamp  = imuData.angularSpeedTimestamp;
    data.angularSpeedX          = imuData.angularSpeedX;
    data.angularSpeedY          = imuData.angularSpeedY;
    data.angularSpeedZ          = imuData.angularSpeedZ;
    data.magneticFieldTimestamp = imuData.magneticFieldTimestamp;
    data.magneticFieldX         = imuData.magneticFieldX;
    data.magneticFieldY         = imuData.magneticFieldY;
    data.magneticFieldZ         = imuData.magneticFieldZ;

    return data;
}

LIS3MDLData HILSensors::getMagnetometerLIS3MDLLastSample()
{
    miosix::PauseKernelLock lock;
    return LIS3MDLData(state.magnetometer->getLastSample(),
                       state.temperature->getLastSample());
}

MS5803Data HILSensors::getMS5803LastSample()
{
    miosix::PauseKernelLock lock;

    auto baroData = state.barometer->getLastSample();
    auto tempData = state.temperature->getLastSample();

    return MS5803Data(baroData.pressureTimestamp, baroData.pressure,
                      tempData.temperatureTimestamp, tempData.temperature);
}

UBXGPSData HILSensors::getUbxGpsLastSample()
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

ADS1118Data HILSensors::getADS1118LastSample()
{
    miosix::PauseKernelLock lock;
    return ads1118 != nullptr ? ads1118->getLastSample() : ADS1118Data{};
}

MPXHZ6130AData HILSensors::getStaticPressureLastSample()
{
    miosix::PauseKernelLock lock;
    return staticPressure != nullptr ? staticPressure->getLastSample()
                                     : MPXHZ6130AData{};
}

SSCDANN030PAAData HILSensors::getDplPressureLastSample()
{
    miosix::PauseKernelLock lock;
    return dplPressure != nullptr ? dplPressure->getLastSample()
                                  : SSCDANN030PAAData{};
}

SSCDRRN015PDAData HILSensors::getPitotPressureLastSample()
{
    miosix::PauseKernelLock lock;
    auto pitotData = pitot != nullptr ? pitot->getLastSample() : PitotData{};
    SSCDRRN015PDAData data;
    data.pressureTimestamp = pitotData.timestamp;
    data.pressure          = pitotData.deltaP;
    return data;
}

PitotData HILSensors::getPitotLastSample()
{
    miosix::PauseKernelLock lock;
    return pitot != nullptr ? pitot->getLastSample() : PitotData{};
}

InternalADCData HILSensors::getInternalADCLastSample()
{
    miosix::PauseKernelLock lock;
    return internalADC != nullptr ? internalADC->getLastSample()
                                  : InternalADCData{};
}

BatteryVoltageSensorData HILSensors::getBatteryVoltageLastSample()
{
    miosix::PauseKernelLock lock;
    return batteryVoltage != nullptr ? batteryVoltage->getLastSample()
                                     : BatteryVoltageSensorData{};
}

void HILSensors::calibrate()
{
    calibrating = true;

    ms5803Stats.reset();
    staticPressureStats.reset();
    dplPressureStats.reset();
    pitotPressureStats.reset();

    bmx160WithCorrection->startCalibration();

    Thread::sleep(CALIBRATION_DURATION);

    bmx160WithCorrection->stopCalibration();

    // Calibrate the analog pressure sensor to the digital one
    float ms5803Mean         = ms5803Stats.getStats().mean;
    float staticPressureMean = staticPressureStats.getStats().mean;
    float dplPressureMean    = dplPressureStats.getStats().mean;
    staticPressure->setOffset(staticPressureMean - ms5803Mean);
    dplPressure->setOffset(dplPressureMean - ms5803Mean);
    pitotPressure->setOffset(pitotPressureStats.getStats().mean);

    calibrating = false;
}

}  // namespace Parafoil
