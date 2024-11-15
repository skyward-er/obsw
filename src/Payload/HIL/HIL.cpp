/* Copyright (c) 2024 Skyward Experimental Rocketry
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

#include "HIL.h"

#include <Payload/Actuators/Actuators.h>
#include <Payload/Buses.h>
#include <Payload/Configs/HILSimulationConfig.h>
#include <Payload/Configs/SensorsConfig.h>
#include <Payload/HIL/HILData.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/StateMachines/WingController/WingController.h>
#include <common/Events.h>
#include <events/EventBroker.h>
#include <hil/HIL.h>
#include <interfaces-impl/hwmapping.h>
#include <utils/DependencyManager/DependencyManager.h>

#include "HILData.h"

namespace Payload
{

PayloadHILPhasesManager::PayloadHILPhasesManager(
    std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition)
    : HILPhasesManager<PayloadFlightPhases, SimulatorData, ActuatorData>(
          getCurrentPosition)
{
    flagsFlightPhases = {{PayloadFlightPhases::SIMULATION_STARTED, false},
                         {PayloadFlightPhases::INITIALIZED, false},
                         {PayloadFlightPhases::CALIBRATION, false},
                         {PayloadFlightPhases::CALIBRATION_OK, false},
                         {PayloadFlightPhases::ARMED, false},
                         {PayloadFlightPhases::LIFTOFF_PIN_DETACHED, false},
                         {PayloadFlightPhases::LIFTOFF, false},
                         {PayloadFlightPhases::MOTOR_SHUTDOWN, false},
                         {PayloadFlightPhases::AEROBRAKES, false},
                         {PayloadFlightPhases::APOGEE, false},
                         {PayloadFlightPhases::PARA1, false},
                         {PayloadFlightPhases::PARA2, false},
                         {PayloadFlightPhases::SIMULATION_STOPPED, false}};

    prev_flagsFlightPhases = flagsFlightPhases;

    // Subscribe to all the topics
    auto& eventBroker = Boardcore::EventBroker::getInstance();
    eventBroker.subscribe(this, Common::TOPIC_ABK);
    eventBroker.subscribe(this, Common::TOPIC_ADA);
    eventBroker.subscribe(this, Common::TOPIC_DPL);
    eventBroker.subscribe(this, Common::TOPIC_CAN);
    eventBroker.subscribe(this, Common::TOPIC_FLIGHT);
    eventBroker.subscribe(this, Common::TOPIC_FMM);
    eventBroker.subscribe(this, Common::TOPIC_FSR);
    eventBroker.subscribe(this, Common::TOPIC_NAS);
    eventBroker.subscribe(this, Common::TOPIC_TMTC);
    eventBroker.subscribe(this, Common::TOPIC_MOTOR);
    eventBroker.subscribe(this, Common::TOPIC_TARS);
    eventBroker.subscribe(this, Common::TOPIC_ALT);
}

void PayloadHILPhasesManager::processFlagsImpl(
    const SimulatorData& simulatorData,
    std::vector<PayloadFlightPhases>& changed_flags)
{
    if (simulatorData.signal ==
        static_cast<float>(HILSignal::SIMULATION_STARTED))
    {
        miosix::reboot();
    }

    if (simulatorData.signal ==
        static_cast<float>(HILSignal::SIMULATION_STOPPED))
    {
        Boardcore::EventBroker::getInstance().post(Common::TMTC_FORCE_LANDING,
                                                   Common::TOPIC_TMTC);
    }

    if (simulatorData.signal ==
        static_cast<float>(HILSignal::SIMULATION_FORCE_LAUNCH))
    {
        Boardcore::EventBroker::getInstance().post(Common::TMTC_ARM,
                                                   Common::TOPIC_TMTC);
        Thread::sleep(250);
        Boardcore::EventBroker::getInstance().post(Common::TMTC_FORCE_LAUNCH,
                                                   Common::TOPIC_TMTC);
    }

    // set true when the first packet from the simulator arrives
    if (isSetTrue(PayloadFlightPhases::SIMULATION_STARTED))
    {
        t_start = Boardcore::TimestampTimer::getTimestamp();

        printf("[HIL] ------- SIMULATION STARTED ! ------- \n");
        changed_flags.push_back(PayloadFlightPhases::SIMULATION_STARTED);
    }
}

void PayloadHILPhasesManager::printOutcomes()
{
    printf("OUTCOMES: (times dt from liftoff)\n\n");
    printf("Simulation time: %.3f [sec]\n\n",
           (double)(t_stop - t_start) / 1000000.0f);

    printf("Motor shutdown: \n");
    outcomes[PayloadFlightPhases::MOTOR_SHUTDOWN].print(t_liftoff);

    printf("Apogee: \n");
    outcomes[PayloadFlightPhases::APOGEE].print(t_liftoff);

    printf("Parachute 1: \n");
    outcomes[PayloadFlightPhases::PARA1].print(t_liftoff);

    printf("Parachute 2: \n");
    outcomes[PayloadFlightPhases::PARA2].print(t_liftoff);

    printf("Simulation Stopped: \n");
    outcomes[PayloadFlightPhases::SIMULATION_STOPPED].print(t_liftoff);
}

void PayloadHILPhasesManager::handleEventImpl(
    const Boardcore::Event& e, std::vector<PayloadFlightPhases>& changed_flags)
{
    switch (e)
    {
        case Common::Events::FMM_INIT_ERROR:
            printf("[HIL] ------- INIT FAILED ! ------- \n");

        case Common::Events::FMM_INIT_OK:
            setFlagFlightPhase(PayloadFlightPhases::INITIALIZED, true);
            TRACE("[HIL] ------- INITIALIZED ! ------- \n");
            changed_flags.push_back(PayloadFlightPhases::INITIALIZED);
            break;
        case Common::Events::FMM_CALIBRATE:
        case Common::Events::CAN_CALIBRATE:
        case Common::Events::TMTC_CALIBRATE:
            setFlagFlightPhase(PayloadFlightPhases::CALIBRATION, true);
            TRACE("[HIL] ------- CALIBRATION ! ------- \n");
            changed_flags.push_back(PayloadFlightPhases::CALIBRATION);
            break;
        case Common::Events::FLIGHT_DISARMED:
            setFlagFlightPhase(PayloadFlightPhases::CALIBRATION_OK, true);
            TRACE("[HIL] ------- CALIBRATION OK ! ------- \n");
            changed_flags.push_back(PayloadFlightPhases::CALIBRATION_OK);
            break;
        case Common::Events::FLIGHT_ARMED:
            setFlagFlightPhase(PayloadFlightPhases::ARMED, true);
            printf("[HIL] ------- READY TO LAUNCH ! ------- \n");
            changed_flags.push_back(PayloadFlightPhases::ARMED);
            break;
        case Common::Events::FLIGHT_LAUNCH_PIN_DETACHED:
            setFlagFlightPhase(PayloadFlightPhases::LIFTOFF_PIN_DETACHED, true);
            TRACE("[HIL] ------- LIFTOFF PIN DETACHED ! ------- \n");
            changed_flags.push_back(PayloadFlightPhases::LIFTOFF_PIN_DETACHED);
            break;
        case Common::Events::FLIGHT_LIFTOFF:
        case Common::Events::TMTC_FORCE_LAUNCH:
            t_liftoff = Boardcore::TimestampTimer::getTimestamp();
            printf("[HIL] ------- LIFTOFF -------: %f, %f \n",
                   getCurrentPosition().z, getCurrentPosition().vz);
            changed_flags.push_back(PayloadFlightPhases::LIFTOFF);
            break;
        case Common::Events::FLIGHT_MOTOR_SHUTDOWN:
            setFlagFlightPhase(PayloadFlightPhases::MOTOR_SHUTDOWN, true);
            registerOutcomes(PayloadFlightPhases::MOTOR_SHUTDOWN);
            printf("[HIL] ------- MOTOR SHUTDOWN ! ------- %f, %f \n",
                   getCurrentPosition().z, getCurrentPosition().vz);
            changed_flags.push_back(PayloadFlightPhases::MOTOR_SHUTDOWN);
            break;
        case Common::Events::FLIGHT_APOGEE_DETECTED:
        case Common::Events::CAN_APOGEE_DETECTED:
        case Common::Events::TMTC_FORCE_APOGEE:
            setFlagFlightPhase(PayloadFlightPhases::AEROBRAKES, false);
            registerOutcomes(PayloadFlightPhases::APOGEE);
            printf("[HIL] ------- APOGEE DETECTED ! ------- %f, %f \n",
                   getCurrentPosition().z, getCurrentPosition().vz);
            changed_flags.push_back(PayloadFlightPhases::APOGEE);
            break;
        case Common::Events::FLIGHT_DROGUE_DESCENT:
        case Common::Events::TMTC_FORCE_EXPULSION:
            setFlagFlightPhase(PayloadFlightPhases::PARA1, true);
            registerOutcomes(PayloadFlightPhases::PARA1);
            printf("[HIL] ------- PARA1 ! -------%f, %f \n",
                   getCurrentPosition().z, getCurrentPosition().vz);
            changed_flags.push_back(PayloadFlightPhases::PARA1);
            break;
        case Common::Events::FLIGHT_WING_DESCENT:
        case Common::Events::FLIGHT_DPL_ALT_DETECTED:
        case Common::Events::TMTC_FORCE_DEPLOYMENT:
            setFlagFlightPhase(PayloadFlightPhases::PARA1, false);
            setFlagFlightPhase(PayloadFlightPhases::PARA2, true);
            registerOutcomes(PayloadFlightPhases::PARA2);
            printf("[HIL] ------- PARA2 ! ------- %f, %f \n",
                   getCurrentPosition().z, getCurrentPosition().vz);
            changed_flags.push_back(PayloadFlightPhases::PARA2);
            break;
        case Common::Events::FLIGHT_LANDING_DETECTED:
        case Common::Events::TMTC_FORCE_LANDING:
            t_stop = Boardcore::TimestampTimer::getTimestamp();
            setFlagFlightPhase(PayloadFlightPhases::PARA2, false);
            setFlagFlightPhase(PayloadFlightPhases::SIMULATION_STOPPED, true);
            changed_flags.push_back(PayloadFlightPhases::SIMULATION_STOPPED);
            registerOutcomes(PayloadFlightPhases::SIMULATION_STOPPED);
            TRACE("[HIL] ------- SIMULATION STOPPED ! -------: %f \n\n\n",
                  (double)t_stop / 1000000.0f);
            printOutcomes();
            break;
        default:
            TRACE("%s event\n", Common::getEventString(e).c_str());
    }
}

PayloadHIL::PayloadHIL()
    : Boardcore::HIL<PayloadFlightPhases, SimulatorData, ActuatorData>(
          nullptr, nullptr, [this]() { return updateActuatorData(); },
          1000 / Config::HIL::SIMULATION_RATE.value())
{
}

bool PayloadHIL::start()
{
    auto* nas      = getModule<Payload::NASController>();
    auto& hilUsart = getModule<Payload::Buses>()->HILUart();

    hilPhasesManager = new PayloadHILPhasesManager(
        [nas]()
        { return Boardcore::TimedTrajectoryPoint(nas->getNasState()); });

    hilTransceiver = new PayloadHILTransceiver(hilUsart, hilPhasesManager);

    return Boardcore::HIL<PayloadFlightPhases, SimulatorData,
                          ActuatorData>::start();
}

ActuatorData PayloadHIL::updateActuatorData()
{
    auto nas       = getModule<Payload::NASController>();
    auto fmm       = getModule<Payload::FlightModeManager>();
    auto wing      = getModule<Payload::WingController>();
    auto actuators = getModule<Payload::Actuators>();

    NASStateHIL nasStateHIL(nas->getNasState());

    ActuatorsStateHIL actuatorsStateHIL(
        actuators->getServoPosition(ServosList::PARAFOIL_LEFT_SERVO),
        actuators->getServoPosition(ServosList::PARAFOIL_RIGHT_SERVO),
        static_cast<float>(miosix::gpios::mainDeploy::value()));

    auto deltaA = actuators->getServoPosition(ServosList::PARAFOIL_LEFT_SERVO) -
                  actuators->getServoPosition(ServosList::PARAFOIL_RIGHT_SERVO);
    deltaA /= 10;  // Mapping to interval [-0.1, 0.1]

    Eigen::Vector2f heading;

    auto psiRef = wing->calculateTargetAngle(
        {nasStateHIL.n, nasStateHIL.e, nasStateHIL.d}, heading);

    GuidanceDataHIL guidanceData(psiRef, deltaA, heading[0], heading[1]);

    // Returning the feedback for the simulator
    return ActuatorData(
        nasStateHIL, actuatorsStateHIL, guidanceData,
        (fmm->testState(&Payload::FlightModeManager::FlyingAscending) ? 3 : 0));
};
}  // namespace Payload