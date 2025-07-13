/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Niccolò Betto, Davide Mor, Davide Basso
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

using namespace Boardcore;
using namespace Boardcore::Units::Time;
using namespace Boardcore::Units::Length;
using namespace Boardcore::Units::Acceleration;
using namespace Eigen;
using namespace Common;
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
}

bool NASController::start()
{
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
            break;
        }

        case NAS_READY:
        {
            transition(&NASController::Ready);
            break;
        }
    }
}

// State skipped on entry because we don't have Armed and Disarmed
// states in the FMM
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
    Sensors* sensors = getModule<Sensors>();

    Vector3f accSum = Vector3f::Zero();
    Vector3f magSum = Vector3f::Zero();
    float baroSum   = 0.0f;

    for (int i = 0; i < config::CALIBRATION_SAMPLES_COUNT; i++)
    {
        BMX160Data imu    = sensors->getBMX160WithCorrectionLastSample();
        PressureData baro = sensors->getLPS22DFLastSample();

        accSum +=
            Vector3f{imu.accelerationX, imu.accelerationY, imu.accelerationZ};
        magSum += Vector3f{imu.magneticFieldX, imu.magneticFieldY,
                           imu.magneticFieldZ};

        baroSum += baro.pressure;

        Thread::sleep(Millisecond{config::CALIBRATION_SLEEP_TIME}.value());
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
    nas.setReferenceValues(reference);

    EventBroker::getInstance().post(NAS_READY, TOPIC_NAS);
}

void NASController::update()
{
    // Update the NAS state only if the FSM is active
    if (state != NASControllerState::ACTIVE)
        return;

    auto* sensors = getModule<Sensors>();

    auto imu  = sensors->getBMX160WithCorrectionLastSample();
    auto gps  = sensors->getUBXGPSLastSample();
    auto baro = sensors->getLPS22DFLastSample();

    // Calculate acceleration
    Vector3f acc   = static_cast<AccelerometerData>(imu);
    auto accLength = MeterPerSecondSquared{acc.norm()};

    miosix::Lock<miosix::FastMutex> l(nasMutex);

    // Perform initial NAS prediction
    nas.predictGyro(imu);
    nas.predictAcc(imu);
    // Disable Magnetometer as it works terribly! DO NOT USE
    // nas.correctMag(imu);

    if (lastGpsTimestamp < gps.gpsTimestamp && gps.fix == 3 &&
        accLength < Config::NAS::DISABLE_GPS_ACCELERATION_THRESHOLD)
    {
        nas.correctGPS(gps);
    }

    // NOTICE: Barometer correction disabled due to broken sensor
    // if (lastBaroTimestamp < baro.pressureTimestamp)
    //     nas.correctBaro(baro.pressure);

    // Correct with accelerometer if the acceleration is in specs
    if (lastAccTimestamp < imu.accelerationTimestamp && acc1g)
        nas.correctAcc(imu);

    // Check if the accelerometer is measuring 1g
    if (accLength < (MeterPerSecondSquared{G{1}} +
                     Config::NAS::ACCELERATION_1G_CONFIDENCE / 2) &&
        accLength > (MeterPerSecondSquared{G{1}} -
                     Config::NAS::ACCELERATION_1G_CONFIDENCE / 2))
    {
        if (acc1gSamplesCount < Config::NAS::ACCELERATION_1G_SAMPLES)
            acc1gSamplesCount++;
        else
            acc1g = true;
    }
    else
    {
        acc1gSamplesCount = 0;
        acc1g             = false;
    }

    lastGyroTimestamp = imu.angularSpeedTimestamp;
    lastAccTimestamp  = imu.accelerationTimestamp;
    lastMagTimestamp  = imu.magneticFieldTimestamp;
    lastGpsTimestamp  = gps.gpsTimestamp;
    lastBaroTimestamp = baro.pressureTimestamp;

    auto state = nas.getState();
    auto ref   = nas.getReferenceValues();

    getModule<FlightStatsRecorder>()->updateNas(state, ref.refTemperature);
    Logger::getInstance().log(state);
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

void NASController::setReferenceAltitude(Meter altitude)
{
    miosix::Lock<miosix::FastMutex> l(nasMutex);

    auto ref        = nas.getReferenceValues();
    ref.refAltitude = altitude.value();
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

}  // namespace Parafoil
