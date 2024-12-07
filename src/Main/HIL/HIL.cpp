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

#include <Main/Actuators/Actuators.h>
#include <Main/Buses.h>
#include <Main/Configs/HILSimulationConfig.h>
#include <Main/Configs/MEAConfig.h>
#include <Main/Configs/SensorsConfig.h>
#include <Main/HIL/HILData.h>
#include <common/Events.h>
#include <events/EventBroker.h>
#include <hil/HIL.h>
#include <interfaces-impl/hwmapping.h>
#include <utils/DependencyManager/DependencyManager.h>

#include "HILData.h"

namespace Main
{

MainHILPhasesManager::MainHILPhasesManager(
    std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition)
    : Boardcore::HILPhasesManager<MainFlightPhases, SimulatorData,
                                  ActuatorData>(getCurrentPosition)
{
    flagsFlightPhases = {{MainFlightPhases::SIM_FLYING, false},
                         {MainFlightPhases::SIM_ASCENT, false},
                         {MainFlightPhases::SIM_BURNING, false},
                         {MainFlightPhases::SIM_AEROBRAKES, false},
                         {MainFlightPhases::SIM_PARA1, false},
                         {MainFlightPhases::SIM_PARA2, false},
                         {MainFlightPhases::SIMULATION_STARTED, false},
                         {MainFlightPhases::CALIBRATION, false},
                         {MainFlightPhases::CALIBRATION_OK, false},
                         {MainFlightPhases::ARMED, false},
                         {MainFlightPhases::LIFTOFF_PIN_DETACHED, false},
                         {MainFlightPhases::LIFTOFF, false},
                         {MainFlightPhases::AEROBRAKES, false},
                         {MainFlightPhases::APOGEE, false},
                         {MainFlightPhases::PARA1, false},
                         {MainFlightPhases::PARA2, false},
                         {MainFlightPhases::SIMULATION_STOPPED, false}};

    prev_flagsFlightPhases = flagsFlightPhases;

    // Subscribe to all the topics
    auto& eventBroker = Boardcore::EventBroker::getInstance();
    eventBroker.subscribe(this, Common::TOPIC_ABK);
    eventBroker.subscribe(this, Common::TOPIC_ADA);
    eventBroker.subscribe(this, Common::TOPIC_MEA);
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

void MainHILPhasesManager::processFlagsImpl(
    const SimulatorData& simulatorData,
    std::vector<MainFlightPhases>& changed_flags)
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
    if (isSetTrue(MainFlightPhases::SIMULATION_STARTED))
    {
        t_start = Boardcore::TimestampTimer::getTimestamp();

        printf("[HIL] ------- SIMULATION STARTED ! ------- \n");
        changed_flags.push_back(MainFlightPhases::SIMULATION_STARTED);
    }

    if (flagsFlightPhases[MainFlightPhases::SIM_FLYING])
    {
        if (isSetTrue(MainFlightPhases::SIM_FLYING))
        {
            registerOutcomes(MainFlightPhases::SIM_FLYING);
            printf("[HIL] ------- SIMULATOR LIFTOFF ! ------- \n");
            changed_flags.push_back(MainFlightPhases::SIM_FLYING);
        }
    }
}

void MainHILPhasesManager::printOutcomes()
{
    printf("OUTCOMES: (times dt from liftoff)\n\n");
    printf("Simulation time: %.3f [sec]\n\n",
           (double)(t_stop - t_start) / 1000000.0f);

    printf("Motor stopped burning (simulation flag): \n");
    outcomes[MainFlightPhases::SIM_BURNING].print(t_liftoff);

    printf("Airbrakes exit shadowmode: \n");
    outcomes[MainFlightPhases::AEROBRAKES].print(t_liftoff);

    printf("Apogee: \n");
    outcomes[MainFlightPhases::APOGEE].print(t_liftoff);

    printf("Parachute 1: \n");
    outcomes[MainFlightPhases::PARA1].print(t_liftoff);

    printf("Parachute 2: \n");
    outcomes[MainFlightPhases::PARA2].print(t_liftoff);

    printf("Simulation Stopped: \n");
    outcomes[MainFlightPhases::SIMULATION_STOPPED].print(t_liftoff);

    // auto cpuMeter = Boardcore::CpuMeter::getCpuStats();
    // printf("max cpu usage: %+.3f\n", cpuMeter.maxValue);
    // printf("avg cpu usage: %+.3f\n", cpuMeter.mean);
    // printf("min free heap: %+.3lu\n", cpuMeter.minFreeHeap);
    // printf("max free stack:%+.3lu\n", cpuMeter.minFreeStack);
}

void MainHILPhasesManager::handleEventImpl(
    const Boardcore::Event& e, std::vector<MainFlightPhases>& changed_flags)
{
    switch (e)
    {
        case Common::Events::FMM_INIT_ERROR:
            printf("[HIL] ------- INIT FAILED ! ------- \n");
        case Common::Events::FMM_INIT_OK:
            setFlagFlightPhase(MainFlightPhases::CALIBRATION, true);
            printf("[HIL] ------- CALIBRATION ! ------- \n");
            changed_flags.push_back(MainFlightPhases::CALIBRATION);
            break;
        case Common::Events::FLIGHT_DISARMED:
            setFlagFlightPhase(MainFlightPhases::CALIBRATION_OK, true);
            printf("[HIL] CALIBRATION OK!\n");
            changed_flags.push_back(MainFlightPhases::CALIBRATION_OK);
            break;
        case Common::Events::FLIGHT_ARMED:
            setFlagFlightPhase(MainFlightPhases::ARMED, true);
            printf("[HIL] ------- READY TO LAUNCH ! ------- \n");
            changed_flags.push_back(MainFlightPhases::ARMED);
            break;
        case Common::Events::FLIGHT_LAUNCH_PIN_DETACHED:
            setFlagFlightPhase(MainFlightPhases::LIFTOFF_PIN_DETACHED, true);
            printf("[HIL] ------- LIFTOFF PIN DETACHED ! ------- \n");
            changed_flags.push_back(MainFlightPhases::LIFTOFF_PIN_DETACHED);
            break;
        case Common::Events::FLIGHT_LIFTOFF:
        case Common::Events::TMTC_FORCE_LAUNCH:
            t_liftoff = Boardcore::TimestampTimer::getTimestamp();
            printf("[HIL] ------- LIFTOFF -------: %f, %f \n",
                   getCurrentPosition().z, getCurrentPosition().vz);
            changed_flags.push_back(MainFlightPhases::LIFTOFF);
            break;
        case Common::Events::FLIGHT_MOTOR_SHUTDOWN:
            printf("[HIL] ------- SHUTDOWN -------: %f, %f \n",
                   getCurrentPosition().z, getCurrentPosition().vz);
            changed_flags.push_back(MainFlightPhases::SHUTDOWN);
        case Common::Events::ABK_SHADOW_MODE_TIMEOUT:
            setFlagFlightPhase(MainFlightPhases::AEROBRAKES, true);
            registerOutcomes(MainFlightPhases::AEROBRAKES);
            printf("[HIL] ABK shadow mode timeout\n");
            changed_flags.push_back(MainFlightPhases::AEROBRAKES);
            break;
        case Common::Events::ADA_SHADOW_MODE_TIMEOUT:
            printf("[HIL] ADA shadow mode timeout\n");
            break;
        case Common::Events::ABK_DISABLE:
            setFlagFlightPhase(MainFlightPhases::AEROBRAKES, false);
            printf("[HIL] ABK disabled\n");
            break;
        case Common::Events::FLIGHT_APOGEE_DETECTED:
        case Common::Events::CAN_APOGEE_DETECTED:
            setFlagFlightPhase(MainFlightPhases::AEROBRAKES, false);
            registerOutcomes(MainFlightPhases::APOGEE);
            printf("[HIL] ------- APOGEE DETECTED ! ------- %f, %f \n",
                   getCurrentPosition().z, getCurrentPosition().vz);
            changed_flags.push_back(MainFlightPhases::APOGEE);
            break;
        case Common::Events::FLIGHT_DROGUE_DESCENT:
        case Common::Events::TMTC_FORCE_EXPULSION:
            setFlagFlightPhase(MainFlightPhases::PARA1, true);
            registerOutcomes(MainFlightPhases::PARA1);
            printf("[HIL] ------- PARA1 ! -------%f, %f \n",
                   getCurrentPosition().z, getCurrentPosition().vz);
            changed_flags.push_back(MainFlightPhases::PARA1);
            break;
        case Common::Events::FLIGHT_WING_DESCENT:
        case Common::Events::FLIGHT_DPL_ALT_DETECTED:
        case Common::Events::TMTC_FORCE_DEPLOYMENT:
            setFlagFlightPhase(MainFlightPhases::PARA1, false);
            setFlagFlightPhase(MainFlightPhases::PARA2, true);
            registerOutcomes(MainFlightPhases::PARA2);
            printf("[HIL] ------- PARA2 ! ------- %f, %f \n",
                   getCurrentPosition().z, getCurrentPosition().vz);
            changed_flags.push_back(MainFlightPhases::PARA2);
            break;
        case Common::Events::FLIGHT_LANDING_DETECTED:
        case Common::Events::TMTC_FORCE_LANDING:
            t_stop = Boardcore::TimestampTimer::getTimestamp();
            setFlagFlightPhase(MainFlightPhases::PARA2, false);
            setFlagFlightPhase(MainFlightPhases::SIMULATION_STOPPED, true);
            changed_flags.push_back(MainFlightPhases::SIMULATION_STOPPED);
            registerOutcomes(MainFlightPhases::SIMULATION_STOPPED);
            printf("[HIL] ------- SIMULATION STOPPED ! -------: %f \n\n\n",
                   (double)t_stop / 1000000.0f);
            printOutcomes();
            break;
        default:
            printf("%s event\n", Common::getEventString(e).c_str());
    }
}

MainHIL::MainHIL()
    : Boardcore::HIL<MainFlightPhases, SimulatorData, ActuatorData>(
          nullptr, nullptr, [this]() { return updateActuatorData(); },
          1000 / Config::HIL::SIMULATION_RATE.value())
{
}

bool MainHIL::start()
{
    auto* nas      = getModule<NASController>();
    auto& hilUsart = getModule<Buses>()->getHILUart();

    hilPhasesManager = new MainHILPhasesManager(
        [nas]()
        { return Boardcore::TimedTrajectoryPoint(nas->getNASState()); });

    hilTransceiver = new MainHILTransceiver(hilUsart, hilPhasesManager);

    return Boardcore::HIL<MainFlightPhases, SimulatorData,
                          ActuatorData>::start();
}

ActuatorData MainHIL::updateActuatorData()
{
    auto actuators = getModule<Actuators>();

    ADAStateHIL adaStateHIL{getModule<ADAController>()->getADAState(),
                            getModule<ADAController>()->getState()};

    NASStateHIL nasStateHIL{getModule<NASController>()->getNASState(),
                            getModule<NASController>()->getState()};

    AirBrakesStateHIL abkStateHIL{getModule<ABKController>()->getState()};

    MEAStateHIL meaStateHIL{getModule<MEAController>()->getMEAState(),
                            getModule<MEAController>()->getState()};

    ActuatorsStateHIL actuatorsStateHIL{
        actuators->getServoPosition(ServosList::AIR_BRAKES_SERVO),
        actuators->getServoPosition(ServosList::EXPULSION_SERVO),
        (actuators->isCanServoOpen(ServosList::MAIN_VALVE) ? 1.f : 0.f),
        (actuators->isCanServoOpen(ServosList::VENTING_VALVE) ? 1.f : 0.f),
        static_cast<float>(miosix::gpios::mainDeploy::value())};

    // Returning the feedback for the simulator
    return ActuatorData{adaStateHIL, nasStateHIL, abkStateHIL, meaStateHIL,
                        actuatorsStateHIL};
}

}  // namespace Main
