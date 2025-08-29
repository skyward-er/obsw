/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano, Pietro Bortolus
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

#include <Main/HIL/HIL.h>
#include <common/CanConfig.h>
#include <common/ReferenceConfig.h>
#include <common/canbus/MotorStatus.h>
#include <drivers/timer/TimestampTimer.h>
#include <sensors/HILSensor.h>
#include <sensors/Sensor.h>

#include "Sensors.h"

namespace Main
{

class HILSensors
    : public Boardcore::InjectableWithDeps<Boardcore::InjectableBase<Sensors>,
                                           MainHIL, Common::MotorStatus>
{
public:
    explicit HILSensors(bool enableHw) : Super{}, enableHw{enableHw} {}

private:
    void lsm6dsrx0Callback() override
    {
        if (!lsm6dsrx_0)
            return;

        Boardcore::Logger::getInstance().log(
            LSM6DSRX0Data{lsm6dsrx_0->getLastSample()});
    }

    void lsm6dsrx1Callback() override
    {
        if (!lsm6dsrx_1)
            return;

        Boardcore::Logger::getInstance().log(
            LSM6DSRX1Data{lsm6dsrx_1->getLastSample()});
    }

    bool postSensorCreationHook() override
    {
        using namespace Boardcore;

        // If full hil, use the can received samples
        if (!getModule<MainHIL>()->isFullHIL())
        {
            // Adding to sensorManager's scheduler a task to "sample" the
            // combustion chamber pressure coming from motor
            getSensorsScheduler().addTask(
                [this]()
                {
                    getModule<Common::MotorStatus>()
                        ->lockData()
                        ->combustionChamberPressure = updateCCData();
                },
                Config::HIL::BARO_CHAMBER_RATE);

            // Adding to sensorManager's scheduler a task to "sample" the
            // pitot static and dynamic pressure coming from payload
            getSensorsScheduler().addTask(
                [this]()
                { setCanPitotStaticPressure(updateStaticPressurePitot()); },
                Config::HIL::BARO_PITOT_RATE);

            getSensorsScheduler().addTask(
                [this]()
                { setCanPitotDynamicPressure(updateDynamicPressurePitot()); },
                Config::HIL::BARO_PITOT_RATE);
        }

        hillificator<>(lps22df, enableHw,
                       [this]() { return updateLPS22DFData(); });
        hillificator<>(h3lis331dl, enableHw,
                       [this]() { return updateH3LIS331DLData(); });
        hillificator<>(lis2mdl, enableHw,
                       [this]() { return updateLIS2MDLData(); });
        hillificator<>(ubxgps, enableHw,
                       [this]() { return updateUBXGPSData(); });
        hillificator<>(lsm6dsrx_0, enableHw,
                       [this]() { return updateLSM6DSRXData_0(); });
        hillificator<>(lsm6dsrx_1, enableHw,
                       [this]() { return updateLSM6DSRXData_1(); });
        hillificator<>(nd015a_0, enableHw,
                       [this]() { return updateStaticPressureData(); });
        hillificator<>(nd015a_1, enableHw,
                       [this]() { return updateStaticPressureData(); });
        hillificator<>(nd015a_2, enableHw,
                       [this]() { return updateStaticPressureData(); });
        hillificator<>(rotatedImu, enableHw,
                       [this]() { return updateIMUData(*this); });

        return true;
    };

    int getSampleCounter(int nData)
    {
        auto ts           = miosix::getTime();
        auto tsSensorData = getModule<MainHIL>()->getTimestampSimulatorData();
        long long simulationPeriod =
            static_cast<long long>(
                getModule<MainHIL>()->getSimulationPeriod()) *
            1e6;

        assert(ts >= tsSensorData &&
               "Actual timestamp is lesser then the packet timestamp");

        if (ts >= tsSensorData + simulationPeriod)
        {
            // TODO: Register this as an error
            return nData - 1;  // Return the last valid index
        }

        // Getting the index floored
        int sampleCounter = (ts - tsSensorData) * nData / simulationPeriod;

        if (sampleCounter < 0)
        {
            printf("sampleCounter: %d\n", sampleCounter);
            assert(sampleCounter < 0 && "Calculated a negative index");
            return 0;
        }

        return sampleCounter;
    }

    Boardcore::LPS22DFData updateLPS22DFData()
    {
        Boardcore::LPS22DFData data;

        auto* sensorData = getModule<MainHIL>()->getSensorData();

        int iBaro = getSampleCounter(sensorData->barometer1.NDATA);
        int iTemp = getSampleCounter(sensorData->temperature.NDATA);

        data.pressureTimestamp = data.temperatureTimestamp =
            Boardcore::TimestampTimer::getTimestamp();
        data.pressure    = sensorData->barometer1.measures[iBaro];
        data.temperature = sensorData->temperature.measures[iTemp];

        return data;
    };

    Boardcore::H3LIS331DLData updateH3LIS331DLData()
    {
        Boardcore::H3LIS331DLData data;

        auto* sensorData = getModule<MainHIL>()->getSensorData();

        int iAcc = getSampleCounter(sensorData->accelerometer1.NDATA);

        data.accelerationTimestamp = Boardcore::TimestampTimer::getTimestamp();
        data.accelerationX = sensorData->accelerometer1.measures[iAcc][0];
        data.accelerationY = sensorData->accelerometer1.measures[iAcc][1];
        data.accelerationZ = sensorData->accelerometer1.measures[iAcc][2];

        return data;
    };

    Boardcore::LIS2MDLData updateLIS2MDLData()
    {
        Boardcore::LIS2MDLData data;

        auto* sensorData = getModule<MainHIL>()->getSensorData();

        int iMag = getSampleCounter(sensorData->magnetometer.NDATA);

        data.magneticFieldTimestamp = Boardcore::TimestampTimer::getTimestamp();
        data.magneticFieldX = sensorData->magnetometer.measures[iMag][0];
        data.magneticFieldY = sensorData->magnetometer.measures[iMag][1];
        data.magneticFieldZ = sensorData->magnetometer.measures[iMag][2];

        return data;
    };

    Boardcore::UBXGPSData updateUBXGPSData()
    {
        Boardcore::UBXGPSData data;

        auto* sensorData = getModule<MainHIL>()->getSensorData();

        int iGps = getSampleCounter(sensorData->gps.NDATA);

        data.gpsTimestamp = Boardcore::TimestampTimer::getTimestamp();

        data.latitude  = sensorData->gps.positionMeasures[iGps][0];
        data.longitude = sensorData->gps.positionMeasures[iGps][1];
        data.height    = sensorData->gps.positionMeasures[iGps][2];

        data.velocityNorth = sensorData->gps.velocityMeasures[iGps][0];
        data.velocityEast  = sensorData->gps.velocityMeasures[iGps][1];
        data.velocityDown  = sensorData->gps.velocityMeasures[iGps][2];
        data.speed         = sqrtf(data.velocityNorth * data.velocityNorth +
                                   data.velocityEast * data.velocityEast +
                                   data.velocityDown * data.velocityDown);
        data.positionDOP   = 0;

        data.fix        = static_cast<uint8_t>(sensorData->gps.fix);
        data.satellites = static_cast<uint8_t>(sensorData->gps.num_satellites);

        return data;
    };

    Boardcore::LSM6DSRXData updateLSM6DSRXData_0()
    {
        Boardcore::LSM6DSRXData data;

        auto* sensorData = getModule<MainHIL>()->getSensorData();

        int iAcc  = getSampleCounter(sensorData->accelerometer1.NDATA);
        int iGyro = getSampleCounter(sensorData->gyro1.NDATA);

        data.accelerationTimestamp = data.angularSpeedTimestamp =
            Boardcore::TimestampTimer::getTimestamp();

        data.accelerationX = sensorData->accelerometer1.measures[iAcc][0];
        data.accelerationY = sensorData->accelerometer1.measures[iAcc][1];
        data.accelerationZ = sensorData->accelerometer1.measures[iAcc][2];

        data.angularSpeedX = sensorData->gyro1.measures[iGyro][0];
        data.angularSpeedY = sensorData->gyro1.measures[iGyro][1];
        data.angularSpeedZ = sensorData->gyro1.measures[iGyro][2];

        return data;
    };

    Boardcore::LSM6DSRXData updateLSM6DSRXData_1()
    {
        Boardcore::LSM6DSRXData data;

        auto* sensorData = getModule<MainHIL>()->getSensorData();

        int iAcc  = getSampleCounter(sensorData->accelerometer2.NDATA);
        int iGyro = getSampleCounter(sensorData->gyro2.NDATA);

        data.accelerationTimestamp = data.angularSpeedTimestamp =
            Boardcore::TimestampTimer::getTimestamp();

        data.accelerationX = sensorData->accelerometer2.measures[iAcc][0];
        data.accelerationY = sensorData->accelerometer2.measures[iAcc][1];
        data.accelerationZ = sensorData->accelerometer2.measures[iAcc][2];

        data.angularSpeedX = sensorData->gyro2.measures[iGyro][0];
        data.angularSpeedY = sensorData->gyro2.measures[iGyro][1];
        data.angularSpeedZ = sensorData->gyro2.measures[iGyro][2];

        return data;
    };

    Boardcore::ND015XData updateStaticPressureData()
    {
        Boardcore::ND015XData data;

        auto* sensorData = getModule<MainHIL>()->getSensorData();

        int iBaro = getSampleCounter(sensorData->barometer1.NDATA);

        data.pressureTimestamp = Boardcore::TimestampTimer::getTimestamp();
        data.pressure          = sensorData->barometer1.measures[iBaro];

        return data;
    };

    Boardcore::IMUData updateIMUData(Main::Sensors& sensors)
    {
        auto imu6 = Config::Sensors::IMU::USE_CALIBRATED_LSM6DSRX
                        ? getCalibratedLSM6DSRX0LastSample()
                        : getLSM6DSRX0LastSample();
        auto mag  = getLIS2MDLLastSample();

        return Boardcore::IMUData{imu6, imu6, mag};
    };

    Boardcore::PressureData updateCCData()
    {
        Boardcore::PressureData data;

        auto* sensorData = getModule<MainHIL>()->getSensorData();

        int iCC = getSampleCounter(sensorData->pressureChamber.NDATA);

        data.pressureTimestamp = Boardcore::TimestampTimer::getTimestamp();
        data.pressure          = sensorData->pressureChamber.measures[iCC];

        return data;
    };

    Boardcore::PressureData updateDynamicPressurePitot()
    {
        Boardcore::PressureData data;

        auto* sensorData = getModule<MainHIL>()->getSensorData();

        int iPitot = getSampleCounter(sensorData->pitot.NDATA);

        data.pressureTimestamp = Boardcore::TimestampTimer::getTimestamp();
        data.pressure          = sensorData->pitot.deltaP[iPitot];

        return data;
    };

    Boardcore::PressureData updateStaticPressurePitot()
    {
        Boardcore::PressureData data;

        auto* sensorData = getModule<MainHIL>()->getSensorData();

        int iPitot = getSampleCounter(sensorData->pitot.NDATA);

        data.pressureTimestamp = Boardcore::TimestampTimer::getTimestamp();
        data.pressure          = sensorData->pitot.staticPressure[iPitot];

        return data;
    };

    bool enableHw;
};
}  // namespace Main
