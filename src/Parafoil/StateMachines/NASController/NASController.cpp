/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Niccolò Betto, Davide Mor
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

#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/NASConfig.h>
#include <Parafoil/FlightStatsRecorder/FlightStatsRecorder.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <algorithms/NAS/StateInitializer.h>
#include <common/Events.h>
#include <common/ReferenceConfig.h>
#include <events/EventBroker.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <chrono>

using namespace Boardcore;
using namespace Eigen;
using namespace Common;
using namespace std::chrono;
namespace config = Parafoil::Config::NAS;

namespace Parafoil
{

NASController::NASController()
    : FSM(&NASController::Init, miosix::STACK_DEFAULT_FOR_PTHREAD,
          BoardScheduler::nasControllerPriority()),
      nas(config::CONFIG)
{
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    altitudeSamples.clear();
}

bool NASController::start()
{
    altitudeSamples.clear();
    // Setup the NAS
    Matrix<float, 13, 1> x = Matrix<float, 13, 1>::Zero();
    // Create the initial quaternion
    Vector4f q = SkyQuaternion::eul2quat({0, 0, 0});

    // Set the initial quaternion inside the matrix
    x(NAS::IDX_QUAT + 0) = q(0);
    x(NAS::IDX_QUAT + 1) = q(1);
    x(NAS::IDX_QUAT + 2) = q(2);
    x(NAS::IDX_QUAT + 3) = q(3);

    // Set the NAS x matrix
    nas.setX(x);
    // Set the initial reference values from the default ones
    nas.setReferenceValues(ReferenceConfig::defaultReferenceValues);

    auto& scheduler = getModule<BoardScheduler>()->nasController();
    // Add the task to the scheduler
    auto task = scheduler.addTask([this] { update(); }, config::UPDATE_RATE,
                                  TaskScheduler::Policy::RECOVER);

    if (task == 0)
    {
        LOG_ERR(logger, "Failed to add NAS update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start NASController FSM active object");
        return false;
    }

    started = true;
    return true;
}

void NASController::initNasdaq()
{
    // Get last NAS state
    Lock<FastMutex> lock{nasMutex};

    // Extract nasdaq config
    NASDAQ0::P_NASDAQ0_T nasdaqConfig = nasdaq.getBlockParameters();

    // Set initial state
    nasdaqConfig.NASStateInterface_InitialCondit[0] = 0;
    nasdaqConfig.NASStateInterface_InitialCondit[1] = 0;
    nasdaqConfig.NASStateInterface_InitialCondit[2] = 0;
    nasdaqConfig.NASStateInterface_InitialCondit[3] = 0;
    nasdaqConfig.NASStateInterface_InitialCondit[4] = 0;
    nasdaqConfig.NASStateInterface_InitialCondit[5] = 0;

    // Set nas covariance
    for (size_t i = 0; i < Config::NASDAQ::NAS_COV_LEN; i++)
        nasdaqConfig.NASVarianceInterface_InitialCon[i] = 0.1;

    // Set modified nasdaq config
    nasdaq.setBlockParameters(&nasdaqConfig);

    // Call the autocoded initialization algorithm
    nasdaq.initialize();
}

NASState NASController::getNasState()
{
    miosix::Lock<miosix::FastMutex> l(nasMutex);
    return nas.getState();
}

ReferenceValues NASController::getReferenceValues()
{
    miosix::Lock<miosix::FastMutex> l(nasMutex);
    return nas.getReferenceValues();
}

NASControllerState NASController::getState() { return state; }

void NASController::setOrientation(const Eigen::Quaternionf& quat)
{
    Lock<FastMutex> lock{nasMutex};

    auto x               = nas.getX();
    x(NAS::IDX_QUAT + 0) = quat.x();
    x(NAS::IDX_QUAT + 1) = quat.y();
    x(NAS::IDX_QUAT + 2) = quat.z();
    x(NAS::IDX_QUAT + 3) = quat.w();
    nas.setX(x);
}

void NASController::Init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(NASControllerState::INIT);
            break;
        }

        case NAS_CALIBRATE:
        {
            transition(&NASController::Calibrating);
            break;
        }
    }
}

void NASController::Calibrating(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(NASControllerState::CALIBRATING);

            calibrate();

            EventBroker::getInstance().post(NAS_READY, TOPIC_NAS);
            break;
        }

        case NAS_READY:
        {
            transition(&NASController::Ready);
            break;
        }
    }
}

void NASController::Ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(NASControllerState::READY);
            transition(&NASController::Active);
            break;
        }
    }
}

void NASController::Active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            initNasdaq();
            updateState(NASControllerState::ACTIVE);
            break;
        }

        case NAS_CALIBRATE:
        {
            transition(&NASController::Calibrating);
            break;
        }

        case FLIGHT_LANDING_DETECTED:
        {
            transition(&NASController::End);
            break;
        }
    }
}

void NASController::End(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateState(NASControllerState::END);
            break;
        }
    }
}

void NASController::calibrate()
{
    altitudeSamples.clear();
    Sensors* sensors = getModule<Sensors>();

    Vector3f accSum = Vector3f::Zero();
    Vector3f magSum = Vector3f::Zero();
    float baroSum   = 0.0f;

    for (int i = 0; i < config::CALIBRATION_SAMPLES_COUNT; i++)
    {
        IMUData imu       = sensors->getIMULastSample();
        PressureData baro = sensors->getStaticPressureLastSample();

        Vector3f acc = static_cast<AccelerometerData>(imu);
        Vector3f mag = static_cast<MagnetometerData>(imu);

        accSum += acc;
        magSum += mag;

        baroSum += baro.pressure;

        Thread::sleep(milliseconds{config::CALIBRATION_SLEEP_TIME}.count());
    }

    Vector3f meanAcc = accSum / config::CALIBRATION_SAMPLES_COUNT;
    meanAcc.normalize();
    Vector3f meanMag = magSum / config::CALIBRATION_SAMPLES_COUNT;
    meanMag.normalize();
    float meanBaro = baroSum / config::CALIBRATION_SAMPLES_COUNT;

    // Use the triad to compute initial state
    StateInitializer init;
    init.triad(meanAcc, meanMag, ReferenceConfig::nedMag);

    miosix::Lock<miosix::FastMutex> l(nasMutex);

    // Compute reference values
    ReferenceValues reference = nas.getReferenceValues();
    reference.refPressure     = meanBaro;
    reference.refAltitude     = Aeroutils::relAltitude(
        reference.refPressure, reference.mslPressure, reference.mslTemperature);

    // Also update the reference with the GPS if we have fix
    UBXGPSData gps = sensors->getUBXGPSLastSample();
    if (gps.fix == 3)
    {
        // Don't use the GPS altitude because it is not reliable
        reference.refLatitude  = gps.latitude;
        reference.refLongitude = gps.longitude;
    }

    // Update the algorithm reference values
    nas.setX(init.getInitX());
    nas.resetCovariance();
    nas.setReferenceValues(reference);
}

void NASController::update()
{
    // Update the NAS state only if the FSM is active
    if (state != NASControllerState::ACTIVE)
        return;

    Boardcore::ADAState ada = getModule<ADAController>()->getADAState();
    const float* covariance = getModule<ADAController>()->getQflattened();
    Sensors* sensors        = getModule<Sensors>();
    auto gps                = sensors->getUBXGPSLastSample();
    auto baro               = sensors->getStaticPressureLastSample();

    // Fill ADA bus
    Bus_AdaState adaBusInput;
    for (size_t i = 0; i < Config::NASDAQ::ADA_DIAG_COV_LEN; i++)
        adaBusInput.covariance[i] = covariance[i];
    adaBusInput.verticalSpeedCovariance =
        getModule<ADAController>()->getVerticalSpeedCov();
    adaBusInput.mslAltitude   = ada.mslAltitude;
    adaBusInput.aglAltitude   = ada.aglAltitude;
    adaBusInput.verticalSpeed = ada.verticalSpeed;
    adaBusInput.x0            = ada.x0;
    adaBusInput.x1            = ada.x1;
    adaBusInput.x2            = ada.x2;
    adaBusInput.apogeeCounter =
        getModule<ADAController>()->getDetectedApogees().ada0DetectedApogees;
    adaBusInput.parachuteCounter = 0;  // Not actually used by the algorithm

    // Fill GPS bus
    Bus_GPS gpsBusInput;
    gpsBusInput.Measure[0] = gps.latitude;
    gpsBusInput.Measure[1] = gps.longitude;
    gpsBusInput.Measure[2] = gps.height;
    gpsBusInput.Measure[3] = gps.velocityNorth;
    gpsBusInput.Measure[4] = gps.velocityEast;
    gpsBusInput.Measure[5] = gps.velocityDown;
    gpsBusInput.Measure[6] = gps.fix;
    gpsBusInput.Measure[7] = gps.satellites;
    gpsBusInput.Measure[8] = gps.speed;
    gpsBusInput.Measure[9] = gps.track;
    gpsBusInput.Timestamp  = gps.gpsTimestamp;

    // Fill baro bus
    Bus_Baro baroBusInput;
    baroBusInput.Measure   = baro.pressure;
    baroBusInput.Timestamp = baro.pressureTimestamp;

    // Run nasdaq
    nasdaq.setADA_States(adaBusInput);
    nasdaq.setGPS(gpsBusInput);
    nasdaq.setBaro(baroBusInput);
    nasdaq.step();

    // Get and log output
    NASDAQ0::ExtY_NASDAQ0_T nasdaqOutput = nasdaq.getExternalOutputs();
    NASDAQState nasdaqState;
    nasdaqState.timestamp = TimestampTimer::getTimestamp();
    nasdaqState.n         = nasdaqOutput.Position[0];
    nasdaqState.e         = nasdaqOutput.Position[1];
    nasdaqState.d         = nasdaqOutput.Position[2];
    nasdaqState.vn        = nasdaqOutput.Velocity[0];
    nasdaqState.ve        = nasdaqOutput.Velocity[1];
    nasdaqState.vd        = nasdaqOutput.Velocity[2];
    nasdaqState.c0        = nasdaqOutput.Covariance[0];
    nasdaqState.c1        = nasdaqOutput.Covariance[1];
    nasdaqState.c2        = nasdaqOutput.Covariance[2];
    nasdaqState.c3        = nasdaqOutput.Covariance[3];
    nasdaqState.c4        = nasdaqOutput.Covariance[4];

    auto ref = nas.getReferenceValues();

    auto altitudeSlm = Aeroutils::relAltitude(baro.pressure, ref.mslPressure,
                                              ref.mslTemperature);
    Meter altitude   = Meter(altitudeSlm) - Meter(ref.refAltitude);

    altitudeSamples.push_back(altitude);
    if (altitudeSamples.size() > 10)
        altitudeSamples.erase(altitudeSamples.begin());

    auto altitudeData =
        AltitudeData{.timestamp   = TimestampTimer::getTimestamp(),
                     .relAltitude = altitude.value()};

    Logger::getInstance().log(altitudeData);
    Logger::getInstance().log(nasdaqState);
    // logging the NAS is done only to not break the logs

    auto nasState = nas.getState();
    getModule<FlightStatsRecorder>()->updateNas(nasState, ref.refTemperature);
    Logger::getInstance().log(nasState);
}

void NASController::updateState(NASControllerState newState)
{
    state = newState;

    auto status = NASControllerStatus{
        .timestamp = TimestampTimer::getTimestamp(),
        .state     = newState,
    };
    Logger::getInstance().log(status);
}

void NASController::setReferenceAltitude(float altitude)
{
    miosix::Lock<miosix::FastMutex> l(nasMutex);

    auto ref        = nas.getReferenceValues();
    ref.refAltitude = altitude;
    nas.setReferenceValues(ref);
}

void NASController::setReferenceTemperature(float temperature)
{
    miosix::Lock<miosix::FastMutex> l(nasMutex);

    auto ref           = nas.getReferenceValues();
    ref.refTemperature = temperature;
    nas.setReferenceValues(ref);
}

void NASController::setReferenceCoordinates(float latitude, float longitude)
{
    miosix::Lock<miosix::FastMutex> l(nasMutex);

    auto ref         = nas.getReferenceValues();
    ref.refLatitude  = latitude;
    ref.refLongitude = longitude;
    nas.setReferenceValues(ref);
}

Meter NASController::getAltitude()
{
    miosix::Lock<miosix::FastMutex> l(nasMutex);

    // The NASDAQ altitude is in NED frame, so it is negative when we are above
    // the reference altitude
    return -Meter{nasdaq.getExternalOutputs().Position[2]};

    // if (altitudeSamples.size() == 0)
    //     return 0.0_m;
    // Meter sum = 0_m;
    // for (auto& sample : altitudeSamples)
    //     sum += sample;
    // return sum / altitudeSamples.size();
}

}  // namespace Parafoil
