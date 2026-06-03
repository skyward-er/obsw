/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor, Pietro Bortolus, Tommaso Lamon
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

#include "NASController.h"

#include <Main/Configs/NASConfig.h>
#include <Main/Configs/SchedulerConfig.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <algorithms/ANAS/StateInitializer.h>
#include <common/Events.h>
#include <common/ReferenceConfig.h>
#include <common/Topics.h>
#include <events/EventBroker.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <algorithm>

using namespace Main;
using namespace Boardcore;
using namespace Common;
using namespace miosix;
using namespace Eigen;

NASController::NASController()
    : FSM{&NASController::state_init, miosix::STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::NAS_PRIORITY},

      anas{}, nasdaq{}
{
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

bool NASController::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getNasScheduler();

    anasID = scheduler.addTask([this]() { updateANAS(); },
                               Config::NAS::UPDATE_RATE_ANAS);

    if (anasID == 0)
    {
        LOG_ERR(logger, "Failed to add ANAS update task");
        return false;
    }

    nasdaqID = scheduler.addTask([this]() { updateNASDAQ(); },
                                 Config::NAS::UPDATE_RATE_NASDAQ);

    if (nasdaqID == 0)
    {
        LOG_ERR(logger, "Failed to add NASDAQ update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start NAS FSM");
        return false;
    }

    anas.initialize();
    nasdaq.initialize();

    scheduler.disableTask(anasID);
    scheduler.disableTask(nasdaqID);

    return true;
}

NASControllerState NASController::getState() { return state; }

ANASState NASController::getANASState()
{
    Lock<FastMutex> lock{nasMutex};

    auto rawOutput = anas.getANAS_Out();

    uint64_t timestamp = TimestampTimer::getTimestamp();

    ANASState state(timestamp, rawOutput.Position, rawOutput.Velocity,
                    rawOutput.Quaternion);

    return state;
}

NASDAQState NASController::getNASDAQState()
{
    Lock<FastMutex> lock{nasMutex};

    auto rawOutput = nasdaq.getNASDAQ_Out();

    uint64_t timestamp = TimestampTimer::getTimestamp();

    NASDAQState state(timestamp, rawOutput.Position, rawOutput.Velocity);
    return state;
}

void NASController::onReferenceChanged(const Boardcore::ReferenceValues& ref)
{
    Lock<FastMutex> l(nasMutex);
    calibrate(ref);
}

void NASController::updateANAS()
{
    if (state == NASControllerState::ACTIVE_ASCENT)
    {
        Lock<FastMutex> lock{nasMutex};

        Sensors* sensors = getModule<Sensors>();

        auto ref          = getModule<AlgoReference>()->getReferenceValues();
        auto imu          = sensors->getIMULastSample();
        auto gps          = sensors->getUBXGPSLastSample();
        auto baro         = sensors->getAtmosPressureLastSample();
        auto staticPitot  = sensors->getCanPitotStaticPressure();
        auto dynamicPitot = sensors->getCanPitotDynamicPressure();

        ANAS0_types_h_::ANASIn inputs = {
            .AccMeasure   = {imu.accelerationX, imu.accelerationY,
                             imu.accelerationZ},
            .AccTimestamp = imu.accelerationTimestamp,

            .GyroMeasure   = {imu.angularSpeedX, imu.angularSpeedY,
                              imu.angularSpeedZ},
            .GyroTimestamp = (imu.angularSpeedTimestamp),

            .BaroMeasure   = baro.pressure,
            .BaroTimestamp = baro.pressureTimestamp,

            .GPSMeasure = {gps.latitude, gps.longitude, gps.velocityNorth,
                           gps.velocityEast},
            .GPSHorizontalPrecision = gps.horizontalAcc,
            .GPSSpeedPrecision      = gps.speedAcc,
            .GPSTimestamp           = gps.gpsTimestamp,

            .PitotMeasure   = {staticPitot.pressure, dynamicPitot.pressure},
            .PitotTimestamp = staticPitot.pressureTimestamp,
            .MagMeasure     = {imu.magneticFieldX, imu.magneticFieldY,
                               imu.magneticFieldZ},
            .MagTimestamp   = {imu.magneticFieldTimestamp}};

        anas.setANAS_In(inputs);
        anas.step();

        ANASLogsData logs(TimestampTimer::getTimestamp(),
                          anas.getANAS_OBSW_Logs());

        getModule<StatsRecorder>()->updateANAS(getANASState());
        sdLogger.log(getANASState());
        sdLogger.log(logs);
    }
}

void NASController::updateNASDAQ()
{
    if (state == NASControllerState::DESCENT)
    {
        Lock<FastMutex> lock{nasMutex};

        Sensors* sensors   = getModule<Sensors>();
        ADAController* ada = getModule<ADAController>();
        ADAState adaState  = ada->getADAState();

        // Pack up inputs
        auto baro = sensors->getAtmosPressureLastSample();
        auto gps  = sensors->getUBXGPSLastSample();

        auto adaVerticalSpeed = adaState.verticalSpeed;
        auto adaTimestamp     = adaState.timestamp;
        auto adaCovariance    = ada->getVerticalSpeedCov();

        NASDAQ0_types_h_::NASDAQInADA ADAIn = {
            .VerticalSpeed           = adaVerticalSpeed,
            .VerticalSpeedCovariance = adaCovariance,
            .Timestamp               = adaTimestamp};

        NASDAQ0_types_h_::NASDAQInSensors sensorIn = {
            .BaroMeasure   = baro.pressure,
            .BaroTimestamp = baro.pressureTimestamp,
            .GPSMeasure    = {gps.latitude, gps.longitude, gps.velocityNorth,
                              gps.velocityEast},
            .GPSTimestamp  = gps.gpsTimestamp};

        // Feed inputs
        nasdaq.setNASDAQ_In_ADA(ADAIn);
        nasdaq.setNASDAQ_In_Sensors(sensorIn);

        // Step
        nasdaq.step();

        // Update and log
        NASDAQLogsWrapper logs(TimestampTimer::getTimestamp(),
                               nasdaq.getNASDAQ_Logs_OBSW());

        getModule<StatsRecorder>()->updateNASDAQ(getNASDAQState());
        sdLogger.log(getNASDAQState());
        sdLogger.log(logs);
    }
}

void NASController::calibrate(const Boardcore::ReferenceValues& ref)
{
    Sensors* sensors = getModule<Sensors>();

    Vector3f accAcc = Vector3f::Zero();
    Vector3f magAcc = Vector3f::Zero();

    // First sample and average the data over a number of samples
    for (int i = 0; i < Config::NAS::CALIBRATION_SAMPLES_COUNT; i++)
    {
        IMUData imu = sensors->getIMULastSample();

        Vector3f acc = static_cast<AccelerometerData>(imu);
        Vector3f mag = static_cast<MagnetometerData>(imu);

        accAcc += acc;
        magAcc += mag;

        Thread::sleep(Config::NAS::CALIBRATION_SLEEP_TIME);
    }

    accAcc /= Config::NAS::CALIBRATION_SAMPLES_COUNT;
    accAcc.normalize();
    magAcc /= Config::NAS::CALIBRATION_SAMPLES_COUNT;
    magAcc.normalize();

    // Use the triad to compute initial state
    StateInitializer init;
    Eigen::Vector4f quat = init.triad(accAcc, magAcc, ReferenceConfig::nedMag);

    ANASReference anasRef = {
        .GroundTemperature = ref.refTemperature,
        .GroundPressure    = ref.refPressure,
        .InitialPosition   = {0, 0, 0},
        .InitialVelocity   = {0, 0, 0},
        .InitialQuaternion = {quat[0], quat[1], quat[2], quat[3]}};

    Lock<FastMutex> lock{nasMutex};
    anas.setANAS_Reference_In(anasRef);

    // NASDAQ setup
    NASDAQReference nasdaqRef = {.GroundTemperature = ref.refTemperature,
                                 .GroundPressure    = ref.refPressure};

    nasdaq.setNASDAQ_In_Reference(nasdaqRef);
}

void NASController::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(NASControllerState::INIT);
            break;
        }

        case NAS_CALIBRATE:
        {
            transition(&NASController::state_calibrating);
            break;
        }
    }
}

void NASController::state_calibrating(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(NASControllerState::CALIBRATING);

            calibrate(getModule<AlgoReference>()->getReferenceValues());

            EventBroker::getInstance().post(NAS_READY, TOPIC_NAS);
            break;
        }

        case NAS_READY:
        {
            transition(&NASController::state_ready);
            break;
        }
    }
}

void NASController::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(NASControllerState::READY);
            break;
        }

        case NAS_RESET:
        {
            // Recalculate initial state with triad via calibration
            [[fallthrough]];
        }
        case NAS_CALIBRATE:
        {
            transition(&NASController::state_calibrating);
            break;
        }

        case NAS_FORCE_START:
        case FLIGHT_ARMED:
        {
            transition(&NASController::state_active_ascent);
            break;
        }
    }
}

void NASController::state_active_ascent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            TaskScheduler& scheduler =
                getModule<BoardScheduler>()->getNasScheduler();

            scheduler.enableTask(anasID);

            updateAndLogStatus(NASControllerState::ACTIVE_ASCENT);
            break;
        }
        case FLIGHT_APOGEE_DETECTED:
        {
            transition(&NASController::state_active_descent);
            break;
        }
        case FLIGHT_LANDING_DETECTED:
        {
            transition(&NASController::state_end);
            break;
        }
        case NAS_FORCE_STOP:
        case FLIGHT_DISARMED:
        {
            transition(&NASController::state_ready);
            break;
        }
    }
}

void NASController::state_active_descent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            TaskScheduler& scheduler =
                getModule<BoardScheduler>()->getNasScheduler();

            nasdaq.setNASDAQ_In_ANAS(anas.getNASDAQ_Initial_State());
            scheduler.enableTask(nasdaqID);
            scheduler.disableTask(anasID);

            updateAndLogStatus(NASControllerState::DESCENT);
            break;
        }
        case FLIGHT_LANDING_DETECTED:
        {
            transition(&NASController::state_end);
            break;
        }
        case NAS_FORCE_STOP:
        case FLIGHT_DISARMED:
        {
            transition(&NASController::state_ready);
        }
    }
}

void NASController::state_end(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(NASControllerState::END);
            break;
        }
    }
}

void NASController::updateAndLogStatus(NASControllerState state)
{
    this->state              = state;
    NASControllerStatus data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}
