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

#include "MEAController.h"

#include <Main/Configs/MEAConfig.h>
#include <Main/Configs/SchedulerConfig.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <events/EventBroker.h>

#include <chrono>

using namespace std::chrono;
using namespace Main;
using namespace Boardcore;
using namespace Common;
using namespace miosix;
using namespace Eigen;

MEA::Config computeMEAConfig()
{
    MEA::Config config;

    // clang-format off
    config.F = Matrix<float, 3, 3>({
        {1.62583090191848f, -0.680722129751093f,	0.0f}, 
        {1.f,                0.f,                0.f},
        {-0.00102053146869855f, 0.000494919888520664f, 1.f}});
    config.Q = Matrix<float, 3, 3>::Identity() * Config::MEA::MODEL_NOISE_VARIANCE;
    config.G = Matrix<float, 3, 1>{{2}, {0}, {0}};
    
    config.baroH = {1.00196621875211f, -0.485916431287183f, 0.0f};
    config.baroR = Config::MEA::SENSOR_NOISE_VARIANCE;

    config.P           = Matrix<float, 3, 3>({
        {0.0f, 0.0f, 0.0f}, 
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.1296f}});
    config.initialMass = Config::MEA::DEFAULT_INITIAL_ROCKET_MASS;
    
    // Trigger acceleration correction at accelerations and speeds 
    // higher then these thresholds
    config.accelThresh = 40.0f; 
    config.speedThresh = 40.0f; 

    // Limits to clamp mass
    config.minMass = 25.0f;
    config.maxMass = 35.0f;

    config.Kt    = 105.2f;
    config.alpha = 0.1057f;
    config.c     = 381.9571f;

    config.coeffs = Config::MEA::AERO_COEFF;
    
    // Rockets diameter
    // TODO: De-hardcode this
    float d = 0.15f;
    config.crossSection = Constants::PI * (d / 2) * (d / 2);
    
    config.ae = 0.00285943f;
    config.p0 = 100093.7492f;

    config.cdCorrectionFactor = Config::MEA::CD_CORRECTION_FACTOR;
    // clang-format on

    return config;
}

MEAController::MEAController()
    : FSM{&MEAController::state_init, miosix::STACK_DEFAULT_FOR_PTHREAD,
          Config::Scheduler::MEA_PRIORITY},
      mea{computeMEAConfig()}
{
    EventBroker::getInstance().subscribe(this, TOPIC_MEA);
    EventBroker::getInstance().subscribe(this, TOPIC_FLIGHT);
}

bool MEAController::start()
{
    TaskScheduler& scheduler = getModule<BoardScheduler>()->getMeaScheduler();

    uint8_t result =
        scheduler.addTask([this]() { update(); }, Config::MEA::UPDATE_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add MEA update task");
        return false;
    }

    if (!FSM::start())
    {
        LOG_ERR(logger, "Failed to start MEA FSM");
        return false;
    }

    return true;
}

MEAState MEAController::getMEAState()
{
    Lock<FastMutex> lock{meaMutex};
    return mea.getState();
}

MEAControllerState MEAController::getState() { return state; }

void MEAController::update()
{
    MEAControllerState curState = state;

    // Lock MEA for the whole duration of the update
    Lock<FastMutex> lock{meaMutex};

    if (curState == MEAControllerState::ARMED ||
        curState == MEAControllerState::SHADOW_MODE ||
        curState == MEAControllerState::ACTIVE ||
        curState == MEAControllerState::ACTIVE_UNPOWERED)
    {
        // Perform updates only during this phases
        PressureData baro;
        IMUData imu         = getModule<Sensors>()->getIMULastSample();
        NASState nas        = getModule<NASController>()->getNASState();
        ReferenceValues ref = getModule<AlgoReference>()->getReferenceValues();

        float mslAltitude = ref.refAltitude - nas.d;
        // TODO: Is this even correct?
        float aperture = 0;

        // Get data from the motor in the smallest scope possible
        // to avoid holding the mutex for too long
        {
            auto motor = getModule<Common::MotorStatus>()->lockData();
            baro       = motor->combustionChamberPressure;
            aperture   = motor->mainValveOpen ? 1.0f : 0.0f;
        }

        if (baro.pressure > Config::MEA::CC_PRESSURE_THRESHOLD)
        {
            MEA::Step step{aperture};

            if (baro.pressureTimestamp > lastBaroTimestamp)
                step.withCCPressure(baro);

            if (nas.timestamp > lastNasTimestamp)
                step.withSpeedAndAlt(-nas.vd, mslAltitude);

            mea.update(step);

            MEAState state = mea.getState();

            // Run detections
            if (curState == MEAControllerState::SHADOW_MODE ||
                curState == MEAControllerState::ACTIVE ||
                curState == MEAControllerState::ACTIVE_UNPOWERED)
            {
                // estimated apogee is msl, so account for that since
                // SHUTDOWN_APOGEE_TARGET is agl
                if (state.estimatedApogee >
                    Config::MEA::SHUTDOWN_APOGEE_TARGET + ref.refAltitude)
                {
                    detectedShutdowns++;
                }
                else
                {
                    detectedShutdowns = 0;
                }

                if (curState == MEAControllerState::ACTIVE)
                {
                    // Throw events only in ACTIVE
                    if (detectedShutdowns >= Config::MEA::SHUTDOWN_N_SAMPLES)
                    {
                        getModule<StatsRecorder>()->shutdownDetected(
                            TimestampTimer::getTimestamp(), mslAltitude);

                        EventBroker::getInstance().post(MEA_SHUTDOWN_DETECTED,
                                                        TOPIC_MEA);
                    }
                }
            }

            lastBaroTimestamp = baro.pressureTimestamp;
            lastAccTimestamp  = imu.accelerationTimestamp;
            lastNasTimestamp  = nas.timestamp;

            sdLogger.log(state);
        }
    }
}

void MEAController::state_init(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(MEAControllerState::INIT);

            // Immediately transition to ready
            transition(&MEAController::state_ready);
            break;
        }
    }
}

void MEAController::state_ready(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(MEAControllerState::READY);
            break;
        }

        case MEA_FORCE_START:
        case FLIGHT_ARMED:
        {
            transition(&MEAController::state_armed);
            break;
        }
    }
}

void MEAController::state_armed(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(MEAControllerState::ARMED);
            break;
        }

        case MEA_FORCE_STOP:
        case FLIGHT_DISARMED:
        {
            transition(&MEAController::state_ready);
            break;
        }

        case FLIGHT_LIFTOFF:
        {
            transition(&MEAController::state_shadow_mode);
            break;
        }
    }
}

void MEAController::state_shadow_mode(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(MEAControllerState::SHADOW_MODE);

            auto shadowModeTime =
                getModule<AlgoReference>()->computeTimeSinceLiftoff(
                    Config::MEA::SHADOW_MODE_TIMEOUT);

            shadowModeTimeoutEvent = EventBroker::getInstance().postDelayed(
                MEA_SHADOW_MODE_TIMEOUT, TOPIC_MEA,
                milliseconds{shadowModeTime}.count());
            break;
        }

        case EV_EXIT:
        {
            EventBroker::getInstance().removeDelayed(shadowModeTimeoutEvent);
            break;
        }

        case MEA_FORCE_STOP:
        {
            transition(&MEAController::state_ready);
            break;
        }

        case MEA_SHADOW_MODE_TIMEOUT:
        {
            transition(&MEAController::state_active);
            break;
        }

        case FLIGHT_MOTOR_SHUTDOWN:
        {
            transition(&MEAController::state_active_unpowered);
            break;
        }

        case FLIGHT_LANDING_DETECTED:
        {
            transition(&MEAController::state_end);
            break;
        }
    }
}

void MEAController::state_active(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(MEAControllerState::ACTIVE);
            break;
        }

        case MEA_FORCE_STOP:
        {
            transition(&MEAController::state_ready);
            break;
        }

        case FLIGHT_MOTOR_SHUTDOWN:
        {
            transition(&MEAController::state_active_unpowered);
            break;
        }

        case FLIGHT_LANDING_DETECTED:
        {
            transition(&MEAController::state_end);
            break;
        }
    }
}

void MEAController::state_active_unpowered(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(MEAControllerState::ACTIVE_UNPOWERED);
            break;
        }

        case MEA_FORCE_STOP:
        {
            transition(&MEAController::state_ready);
            break;
        }

        case FLIGHT_APOGEE_DETECTED:
        case FLIGHT_LANDING_DETECTED:
        {
            transition(&MEAController::state_end);
            break;
        }
    }
}

void MEAController::state_end(const Event& event)
{
    switch (event)
    {
        case EV_ENTRY:
        {
            updateAndLogStatus(MEAControllerState::END);
            break;
        }
    }
}

void MEAController::updateAndLogStatus(MEAControllerState state)
{
    this->state              = state;
    MEAControllerStatus data = {TimestampTimer::getTimestamp(), state};
    sdLogger.log(data);
}
