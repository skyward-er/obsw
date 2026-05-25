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
      nasdaq{}, reference{ReferenceConfig::defaultReferenceValues}
{
    EventBroker::getInstance().subscribe(this, TOPIC_NAS);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
    altitudeSamples.clear();
}

bool NASController::start()
{
    altitudeSamples.clear();

    auto& scheduler = getModule<BoardScheduler>()->nasController();
    // Add the task to the scheduler
    auto task =
        scheduler.addTask([this] { update(); }, config::UPDATE_RATE_ALTITUDE,
                          TaskScheduler::Policy::RECOVER);
    if (task == 0)
        LOG_ERR(logger, "Failed to add NAS update task");

    nasdaqID = scheduler.addTask([this]() { updateNASDAQ(); },
                                 Config::NAS::UPDATE_RATE_NASDAQ);

    if (nasdaqID == 0)
    {
        LOG_ERR(logger, "Failed to add NASDAQ update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start NASController FSM active object");
        return false;
    }

    // scheduler.disableTask(nasdaqID);
    return true;
}

void NASController::initNASDAQ()
{
    ANAS_NASDAQ ANASOutNASDAQIn = {
        /* clang-format off */
        .LinearCovariance = {0.01,    0,    0,    0,    0,    0, 
                                0, 0.01,    0,    0,    0,    0,
                                0,    0, 0.01,    0,    0,    0,
                                0,    0,    0, 0.01,    0,    0,
                                0,    0,    0,    0, 0.01,    0,
                                0,    0,    0,    0,    0, 0.01,
                            },
        /* clang-format on */
        .Position = {0, 0, 0},
        .Velocity = {0, 0, 0}};

    nasdaq.initialize();
    nasdaq.setNASDAQ_In_ANAS(ANASOutNASDAQIn);
    nasdaq.setNASDAQ_In_Reference(
        NASDAQReference{.GroundTemperature = reference.refTemperature,
                        .GroundPressure    = reference.refPressure});
}

NASDAQState NASController::getNASDAQState()
{
    Lock<FastMutex> lock{nasMutex};

    auto rawOutput = nasdaq.getNASDAQ_Out();

    uint64_t timestamp = TimestampTimer::getTimestamp();

    NASDAQState state(timestamp, rawOutput.Position, rawOutput.Velocity);

    return state;
}

ReferenceValues NASController::getReferenceValues()
{
    miosix::Lock<miosix::FastMutex> l(nasMutex);
    return reference;
}

NASControllerState NASController::getState() { return state; }

void NASController::updateNASDAQ()
{
    if (state == NASControllerState::ACTIVE)
    {
        Lock<FastMutex> lock{nasMutex};

        Sensors* sensors      = getModule<Sensors>();
        ADAController* adaRef = getModule<ADAController>();
        if (sensors->getUBXGPSLastSample().fix != 3)
            return;  // we do not run NASDAQ when we loose the GPS fix

        // Pack up inputs
        auto baro =
            sensors
                ->getStaticPressureLastSample();  // check for struct
                                                  // alignment with chad,
                                                  // might need to break it up
        auto gps = sensors->getUBXGPSLastSample();

        auto adaState = adaRef->getADAState();

        auto adaVerticalSpeed = adaState.verticalSpeed;

        auto adaCovariance = adaRef->getVerticalSpeedCov();

        NASDAQ0_types_h_::NASDAQInADA ADAIn = {
            .VerticalSpeed           = adaVerticalSpeed,
            .VerticalSpeedCovariance = adaCovariance,
            .Timestamp               = adaState.timestamp};

        Logger::getInstance().log(NASDAQADAInData{ADAIn});

        NASDAQ0_types_h_::NASDAQInSensors sensorIn = {
            .BaroMeasure   = baro.pressure,
            .BaroTimestamp = baro.pressureTimestamp,
            .GPSMeasure    = {gps.latitude, gps.longitude, gps.velocityNorth,
                              gps.velocityEast},
            .GPSTimestamp  = gps.gpsTimestamp};

        Logger::getInstance().log(NASDAQSensorInData{sensorIn});

        // Feed inputs

        nasdaq.setNASDAQ_In_ADA(ADAIn);
        nasdaq.setNASDAQ_In_Sensors(sensorIn);

        // Step
        nasdaq.step();
    }

    // Update and log

    NASDAQLogsWrapper logs(TimestampTimer::getTimestamp(),
                           nasdaq.getNASDAQ_Logs_OBSW());

    Logger::getInstance().log(logs);

    // Probabilmente aggiornare NASDAQ con gli input dell'ANAS in Entry
    // dello stato della state

    getModule<FlightStatsRecorder>()->updateNASDAQ(getNASDAQState());
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
            updateState(NASControllerState::ACTIVE);
            initNASDAQ();
            break;
        }

        case NAS_CALIBRATE:
        {
            transition(&NASController::Calibrating);
            break;
        }

        case FLIGHT_WING_DESCENT:
        {
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

    float baroSum = 0.0f;

    for (int i = 0; i < config::CALIBRATION_SAMPLES_COUNT; i++)
    {
        PressureData baro = sensors->getStaticPressureLastSample();

        baroSum += baro.pressure;

        Thread::sleep(milliseconds{config::CALIBRATION_SLEEP_TIME}.count());
    }

    float meanBaro = baroSum / config::CALIBRATION_SAMPLES_COUNT;

    miosix::Lock<miosix::FastMutex> l(nasMutex);

    // Compute reference values
    reference.refPressure = meanBaro;
    reference.refAltitude = Aeroutils::relAltitude(
        reference.refPressure, reference.mslPressure, reference.mslTemperature);

    // Also update the reference with the GPS if we have fix
    UBXGPSData gps = sensors->getUBXGPSLastSample();
    if (gps.fix == 3)
    {
        // Don't use the GPS altitude because it is not reliable
        reference.refLatitude  = gps.latitude;
        reference.refLongitude = gps.longitude;
    }

    initNASDAQ();
}

void NASController::update()
{
    // Update the NAS state only if the FSM is active
    if (state != NASControllerState::ACTIVE)
        return;

    Sensors* sensors = getModule<Sensors>();
    auto baro        = sensors->getStaticPressureLastSample();

    auto altitudeSlm = Aeroutils::relAltitude(
        baro.pressure, reference.mslPressure, reference.mslTemperature);
    Meter altitude = Meter(altitudeSlm) - Meter(reference.refAltitude);

    altitudeSamples.push_back(altitude);
    if (altitudeSamples.size() > 10)
        altitudeSamples.erase(altitudeSamples.begin());

    auto altitudeData =
        AltitudeData{.timestamp   = TimestampTimer::getTimestamp(),
                     .relAltitude = altitude.value()};

    Logger::getInstance().log(altitudeData);
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

    reference.refAltitude = altitude;
}

void NASController::setReferenceTemperature(float temperature)
{
    miosix::Lock<miosix::FastMutex> l(nasMutex);

    reference.refTemperature = temperature;
}

void NASController::setReferenceCoordinates(float latitude, float longitude)
{
    miosix::Lock<miosix::FastMutex> l(nasMutex);
    reference.refLatitude  = latitude;
    reference.refLongitude = longitude;
}

Meter NASController::getAltitude()
{
    miosix::Lock<miosix::FastMutex> l(nasMutex);

#ifdef USE_NASDAQ
    // The NASDAQ altitude is in NED frame, so it is negative when we are
    // above the reference altitude
    return -Meter{nasdaq.getNASDAQ_Out().Position[2]};
#else
    if (altitudeSamples.size() == 0)
        return 0.0_m;
    Meter sum = 0_m;
    for (auto& sample : altitudeSamples)
        sum += sample;
    return sum / altitudeSamples.size();
#endif
}

}  // namespace Parafoil
