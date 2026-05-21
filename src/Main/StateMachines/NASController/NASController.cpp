/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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
#include <algorithms/NAS/StateInitializer.h>
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

        anas{},
        nasdaq{}
{
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

bool NASController::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getNasScheduler();


    anasID =
        scheduler.addTask([this]() { updateANAS(); }, Config::NAS::UPDATE_RATE_ANAS);

    if (anasID == 0)
    {
        LOG_ERR(logger, "Failed to add ANAS update task");
        return false;
    }

    nasdaqID = scheduler.addTask([this]() { updateNASDAQ(); }, Config::NAS::UPDATE_RATE_NASDAQ); 

    if (nasdaqID == 0) {
        LOG_ERR(logger, "Failed to add NASDAQ update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start NAS FSM");
        return false;
    }

    // Verrà cannonato, tutto questo verrà fatto dentro calibrate

    // Initialize reference
    auto algoRef        = getModule<AlgoReference>();
    ReferenceValues ref = algoRef->getReferenceValues();
    nas.setReferenceValues(ref);

    algoRef->subscribeReferenceChanges(this);

    // Initialize state
    Matrix<float, 13, 1> x = Matrix<float, 13, 1>::Zero();
    Vector4f q             = SkyQuaternion::eul2quat({0, 0, 0});

    x(6) = q(0);
    x(7) = q(1);
    x(8) = q(2);
    x(9) = q(3);

    nas.setX(x);


    scheduler.disableTask(anasID);
    scheduler.disableTask(nasdaqID);

    return true;
}

NASControllerState NASController::getState() { return state; }

ANASState NASController::getANASState()
{
    Lock<FastMutex> lock{nasMutex};

    auto rawOutput = anas.getNASOut();

    // Devo passare questo o il timestamp interno?
    uint64_t timestamp = miosix::getTime();

    ANASState state(timestamp, rawOutput.Position, rawOutput.Velocity,
                    rawOutput.Quaternion);

    return state;
}

NASDAQState NASController::getNASDAQState()
{
    Lock<FastMutex> lock{nasMutex};

    auto rawOutput = nasdaq.getNASDAQ_Out();

    uint64_t timestamp = miosix::getTime();

    NASDAQState state(timestamp, rawOutput.Position, rawOutput.Velocity);
}

// TODO Aggiungere getter per ANAS e NASDAQ

void NASController::setOrientation(Eigen::Quaternion<float> quat)
{
    // Need to lock mutex because the only invocation comes from the radio
    // which is a separate thread
    Lock<FastMutex> lock{nasMutex};

    Matrix<float, 13, 1> x = nas.getX();
    x(6)                   = quat.x();
    x(7)                   = quat.y();
    x(8)                   = quat.z();
    x(9)                   = quat.w();
    nas.setX(x);
}

// Da cambiare
void NASController::onReferenceChanged(const ReferenceValues& ref)
{
    Lock<FastMutex> l(nasMutex);
    nas.setReferenceValues(ref);
}

// Set Block Parameters e reference ANAS
void NASController::onANASReferenceChanged() {}

void NASController::onNASDAQReferenceChanged() {}


void NASController::updateANAS()
{

    if (state == NASControllerState::ACTIVE_ASCENT)
    {

        Lock<FastMutex> lock{nasMutex};

        Sensors* sensors = getModule<Sensors>();

        auto prevState    = getANASState();
        auto ref          = getModule<AlgoReference>()->getReferenceValues();
        float mslAltitude = ref.refAltitude - prevState.d;
        float mach        = Aeroutils::computeMach(-mslAltitude, -prevState.vd,
                                                   ref.mslTemperature);
        auto imu          = sensors->getIMULastSample();
        auto gps          = sensors->getUBXGPSLastSample();
        auto baro         = sensors->getAtmosPressureLastSample();
        auto staticPitot  = sensors->getCanPitotStaticPressure();
        auto dynamicPitot = sensors->getCanPitotDynamicPressure();

        ANAS0_types_h_::NASIn inputs = {
            .AccMeasure   = {imu.accelerationX, imu.accelerationY,
                             imu.accelerationZ},
            .AccTimestamp = imu.accelerationTimestamp,

            .GyroMeasure   = {imu.angularSpeedX, imu.angularSpeedY,
                              imu.angularSpeedZ},
            .GyroTimestamp = (imu.angularSpeedTimestamp),

            .BaroMeasure   = baro.pressure,
            .BaroTimestamp = baro.pressureTimestamp,

            .GPSMeasure = {gps.latitude, gps.longitude, gps.height, gps.speed},
            .GPSTimestamp     = gps.gpsTimestamp,
            .GPSHorizAccuracy = gps.hAcc,
            .GPSVertAccuracy  = gps.sAcc,

            .PitotMeasure   = {staticPitot.pressure, dynamicPitot.pressure},
            .PitotTimestamp = staticPitot.pressureTimestamp,
            .MagMeasure     = {imu.magneticFieldX, imu.magneticFieldY,
                               imu.magneticFieldZ},
            .MagTimestamp   = {imu.magneticFieldTimestamp}};

        anas.setNASIn(inputs);
        anas.step();

        ANASLogsData logs(miosix::getTime(), anas.getNASLogs());

        getModule<StatsRecorder>()->updateANAS(getANASState());

        sdLogger.log(getANASState());
        sdLogger.log(logs);
    }
}

void NASController::updateNASDAQ() {
    
    if (state == NASControllerState::DESCENT)
    {

        Lock<FastMutex> lock{nasMutex};

        Sensors* sensors      = getModule<Sensors>();
        ADAController* adaRef = getModule<ADAController>();

        // Pack up inputs
        auto baro =
            sensors->getAtmosPressureLastSample();  // check for struct
                                                    // alignment with chad,
                                                    // might need to break it up
        auto gps = sensors->getUBXGPSLastSample();

        auto adaVerticalSpeed =
            adaRef->getMaxVerticalSpeed();  // Check if this is the correct data
                                            // wanted by GNC

        auto adaTimestamp  = adaRef->getADAStateTemp().timestamp;
        auto adaCovariance = adaRef->getVerticalVelocityCovariance();

        NASDAQ0_types_h_::NASDAQInADA ADAIn = {
            .VerticalSpeed           = adaVerticalSpeed,
            .VerticalSpeedCovariance = adaCovariance,
            .Timestamp               = miosix::getTime()};

        NASDAQ0_types_h_::NASDAQInSensors sensorIn = {
            .BaroMeasure   = baro.pressure,
            .BaroTimestamp = baro.pressureTimestamp,
            .GPSMeasure = {gps.latitude, gps.longitude, gps.height, gps.speed},
            .GPSTimestamp = gps.gpsTimestamp};

        // Feed inputs

        nasdaq.setNASDAQ_In_ADA(ADAIn);
        nasdaq.setNASDAQ_In_Sensors(sensorIn);

        // Step
        nasdaq.step();

        // Update and log

        NASDAQLogsWrapper logs(miosix::getTime(), nasdaq.getNASDAQ_Logs_OBSW());

        sdLogger.log(getNASDAQState());
        sdLogger.log(logs);

        // Probabilmente aggiornare NASDAQ con gli input dell'ANAS in Entry
        // dello stato della state

        getModule<StatsRecorder>()->updateNASDAQ(getNASDAQState());
    }
}

// Serve, ma ho bisogno dell'anas per caricare i reference values
// Da rivedere
void NASController::calibrate()
{

    // Aggiungere posizione e velocità (file di config a zero) quaternioni invece da triad e setta i param iniziali
    // Set stato e covarianza
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
    init.triad(accAcc, magAcc, ReferenceConfig::nedMag);

    ReferenceValues ref = getModule<AlgoReference>()->getReferenceValues();

    Lock<FastMutex> lock{nasMutex};
    nas.setX(init.getInitX());
    nas.setReferenceValues(ref);
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

            calibrate();

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
            Lock<FastMutex> l(nasMutex);
            nas.resetCovariance();

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
            TaskScheduler& scheduler = getModule<BoardScheduler>()->getNasScheduler();

            scheduler.enableTask(anasID);

            updateAndLogStatus(NASControllerState::ACTIVE_ASCENT);
            break;
        }
        case FLIGHT_APOGEE_DETECTED:
        {
            transition(&NASController::state_descent);
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

void NASController::state_descent(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {

            TaskScheduler& scheduler = getModule<BoardScheduler>()->getNasScheduler();

            ANAS_NASDAQ ANASOutNASDAQIn = {

                .LinearCovariance = *anas.getNASFinal().LinearCovariance,
                .Position         = *anas.getNASOut().Position,
                .Velocity         = *anas.getNASOut().Velocity};

            nasdaq.setNASDAQ_In_ANAS(ANASOutNASDAQIn);
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
    NASControllerStatus data = {miosix::getTime(), state};
    sdLogger.log(data);
}
