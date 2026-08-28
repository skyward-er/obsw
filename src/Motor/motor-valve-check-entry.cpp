/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Riccardo Sironi
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

#include <Motor/Actuators/Actuators.h>
#include <Motor/BoardScheduler.h>
#include <Motor/Buses.h>
#include <Motor/CanHandler/CanHandler.h>
#include <Motor/PersistentVars/PersistentVars.h>
#include <Motor/Sensors/Sensors.h>
#include <Motor/StateMachines/EregController/EregControllerFuel.h>
#include <Motor/StateMachines/EregController/EregControllerOx.h>
#include <Motor/StateMachines/FiringSequenceHSM/FiringSequenceHSM.h>
#include <Motor/StateMachines/MEAController/MEAController.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <chrono>
#include <iomanip>
#include <iostream>

using namespace std::chrono;
using namespace Boardcore;
using namespace Motor;
using namespace Common;
using namespace miosix;

namespace Motor
{
enum class StatusBit
{
    MODULES,
    SD_LOGGER,
    SCHEDULER,
    ACTUATORS,
    CAN_HANDLER,
    SENSORS,
    LAST = 7,  // Last status bit since flag is 8 bits
};

static uint8_t statusFlags = 0;

void setStatus(StatusBit bit)
{
    statusFlags |= (1 << static_cast<uint8_t>(bit));
}

struct ValveEntry
{
    const char* name;
    ServosList valve;
};
static const ValveEntry valveEntries[] = {
    // {"OX_VENTING", OX_VENTING_VALVE}, {"FUEL_VENTING", FUEL_VENTING_VALVE},
    {"MAIN_FUEL", MAIN_FUEL_VALVE},
    {"MAIN_OX", MAIN_OX_VALVE},
    {"PRZ_FUEL", PRZ_FUEL_VALVE},
    {"PRZ_OX", PRZ_OX_VALVE},

};
ServosList getValveFromString(const std::string& name)
{
    if (name.empty())
        return ServosList_ENUM_END;

    bool isNumeric = true;
    for (char c : name)
    {
        if (c < '0' || c > '9')
        {
            isNumeric = false;
            break;
        }
    }

    if (isNumeric)
    {
        int num = std::stoi(name);
        if (num > 0 && num < ServosList_ENUM_END)
            return static_cast<ServosList>(num);
        std::cerr << "Valve number " << num << " is out of range (1 - "
                  << (ServosList_ENUM_END - 1) << ")" << std::endl;
        return ServosList_ENUM_END;
    }

    for (const auto& entry : valveEntries)
        if (name == entry.name)
            return entry.valve;

    return ServosList_ENUM_END;
}

void printValveList()
{
    std::cout << "Available valves:" << std::endl;
    for (const auto& entry : valveEntries)
    {
        std::cout << "  " << std::setw(3) << static_cast<int>(entry.valve)
                  << "  " << entry.name << std::endl;
    }
}

std::vector<std::string> split(const std::string& str)
{
    std::vector<std::string> tokens;
    std::istringstream iss(str);
    std::string token;

    while (iss >> token)
        tokens.push_back(token);

    return tokens;
}
}  // namespace Motor
int main()
{
    ledOff();

    bool initResult = true;

    DependencyManager manager;

    Buses* buses              = new Buses();
    BoardScheduler* scheduler = new BoardScheduler();

    Sensors* sensors       = nullptr;
    auto actuators         = new Actuators();
    auto eregOx            = new EregControllerOx();
    auto eregFuel          = new EregControllerFuel();
    auto registry          = new Registry();
    auto firingSequenceHSM = new FiringSequenceHSM();
    auto meaController     = new MEAController();
    auto canHandler        = new CanHandler();
    auto& sdLogger         = Logger::getInstance();

    sensors = new Sensors();

    initResult &= manager.insert<Buses>(buses) &&
                  manager.insert<BoardScheduler>(scheduler) &&
                  manager.insert<Registry>(registry) &&
                  manager.insert<Sensors>(sensors) &&
                  manager.insert<Actuators>(actuators) &&
                  manager.insert<EregControllerOx>(eregOx) &&
                  manager.insert<EregControllerFuel>(eregFuel) &&
                  manager.insert<FiringSequenceHSM>(firingSequenceHSM) &&
                  manager.insert<MEAController>(meaController) &&
                  manager.insert<CanHandler>(canHandler) && manager.inject();

    if (!initResult)
    {
        std::cerr << "*** Failed to inject dependencies ***" << std::endl;
        return -1;
    }
    else
    {
        setStatus(StatusBit::MODULES);
    }

    // Status led indicators
    // led1: Sensors init/error
    // led2: Actuators init/error
    // led3: CanBus init/error
    // led4: Everything ok

    // Start logging when system boots
    std::cout << "Starting Logger" << std::endl;
    if (!sdLogger.start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Logger ***" << std::endl;

        if (!sdLogger.testSDCard())
            std::cerr << "\tReason: SD card not present or not writable"
                      << std::endl;
        else
            std::cerr << "\tReason: Logger initialization error" << std::endl;
    }
    else
    {
        sdLogger.resetStats();
        std::cout << "Logger Ok!\n"
                  << "\tLog number: " << sdLogger.getStats().logNumber
                  << std::endl;
        setStatus(StatusBit::SD_LOGGER);
    }

    std::cout << "Starting BoardScheduler" << std::endl;
    if (!scheduler->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start BoardScheduler ***" << std::endl;
    }
    else
    {
        setStatus(StatusBit::SCHEDULER);
    }

    std::cout << "Starting Actuators" << std::endl;
    led2On();
    if (!actuators->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Actuators ***" << std::endl;
    }
    else
    {
        led2Off();
        setStatus(StatusBit::ACTUATORS);
    }

    std::cout << "Starting CanHandler" << std::endl;
    led3On();
    if (!canHandler->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start CanHandler ***" << std::endl;
    }
    else
    {
        led3Off();
        setStatus(StatusBit::CAN_HANDLER);
    }

    std::cout << "Starting Registry" << std::endl;
    if (!registry->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Registry ***" << std::endl;
    }

    // Perform an initial registry load
    std::cout << "Loading backed registry" << std::endl;
    if (registry->load() != RegistryError::OK)
        std::cout << "* Warning: could not load a saved registry *"
                  << std::endl;

    std::cout << "Starting FiringSequenceHSM" << std::endl;
    if (!firingSequenceHSM->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start FiringSequenceHSM ***" << std::endl;
    }

    std::cout << "Starting eregControllerOX" << std::endl;
    if (!eregOx->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start eregControllerOx ***" << std::endl;
    }

    std::cout << "Starting eregControllerFuel" << std::endl;
    if (!eregFuel->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start eregControllerFuel ***" << std::endl;
    }

    std::cout << "Starting Sensors" << std::endl;
    led1On();
    if (!sensors->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Sensors ***" << std::endl;
    }
    else
    {
        std::cout << "\tCalibrating sensors" << std::endl;
        sensors->calibrate();
        led1Off();
        setStatus(StatusBit::SENSORS);
    }
    if (initResult)
    {
        // canHandler->setInitStatus(statusFlags);
        std::cout << "All good!" << std::endl;
        led4On();
    }
    else
    {
        // canHandler->setInitStatus(statusFlags);
        std::cerr << "*** Init failure ***" << std::endl;
    }

    std::cout << "Sensor status:" << std::endl;
    for (auto info : sensors->getSensorInfos())
    {
        // The period being 0 means the sensor is disabled
        auto statusStr = info.period == 0ns   ? "Disabled"
                         : info.isInitialized ? "Ok"
                                              : "Error";

        std::cout << "\t" << std::setw(24) << std::left << info.id << " "
                  << statusStr << std::endl;
    }

    std::cout << "Battery voltage: " << std::fixed << std::setprecision(2)
              << sensors->getBatteryVoltage().voltage << " V" << std::endl;

    std::cout << "Servo Current Consumption : " << std::fixed
              << std::setprecision(2)
              << sensors->getServoCurrentConsumption().current << " A"
              << std::endl;
    std::cout << "Igniter Current Consumption : " << std::fixed
              << std::setprecision(2)
              << sensors->getIgniterCurrentConsumption().current << " A"
              << std::endl;
    std::cout << "Solenoid Current Consumption : " << std::fixed
              << std::setprecision(2)
              << sensors->getSolenoidCurrentConsumption().current << " A"
              << std::endl;

    // From here on main thread will do non-critical stuff, set lowest priority
    Thread::setPriority(BoardScheduler::Priority::LOW);

    while (true)
    {
        // Log logger and CPU stats
        sdLogger.log(sdLogger.getStats());
        sdLogger.log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();

        // Toggle LED
        gpios::debugLedGreen::value() ? gpios::debugLedGreen::low()
                                      : gpios::debugLedGreen::high();
        // gpios::debugLedOrange::value() ? gpios::debugLedOrange::low()
        //                                : gpios::debugLedOrange::high();
        // gpios::debugLedYellow::value() ? gpios::debugLedYellow::low()
        //                                : gpios::debugLedYellow::high();
        // gpios::debugLedRed::value() ? gpios::debugLedRed::low()
        //                             : gpios::debugLedRed::high();

        std::cout << "Enter the movement command:\n";
        std::cout << "Format: animate <VALVE_NAME|number> to <POSITION> in "
                     "<MILLISECONDS>\n";
        std::cout << "or : step <VALVE_NAME|number>\n";
        std::cout << "or : list for a list of available valves\n";

        std::string valveCmd;
        std::getline(std::cin, valveCmd);

        auto words = split(valveCmd);

        if (words.empty())
            continue;

        if (words[0] == "list")
        {
            printValveList();
        }
        else
        {
            if (words.size() < 2)
            {
                std::cout << "\nInvalid command: " << valveCmd << std::endl;
                continue;
            }
            ServosList valve = getValveFromString(words[1]);

            // std::cout << "Valve: " << words[1] << " -> "
            //           << static_cast<int>(valve) << std::endl;

            if (valve != ServosList::ServosList_ENUM_END)
            {
                if (words[0] == "animate")
                {
                    if (words.size() < 6 || words[2] != "to" ||
                        words[4] != "in")
                    {
                        std::cout << "\nInvalid command: " << valveCmd
                                  << std::endl;
                        continue;
                    }
                    actuators->closeValve(valve);

                    float position = std::stof(words[3]);
                    int duration   = std::stoi(words[5]);
                    std::cout << "Sweeping valve " << words[1]
                              << " to position " << position << " in "
                              << duration << " ms" << std::endl;
                    actuators->animateValve(valve, position, duration);
                }
                else if (words[0] == "step")
                {
                    actuators->closeValve(valve);
                    std::cout << "\nStepping valve " << words[1] << std::endl;
                    for (float i = 0; i < 10; i++)
                    {
                        float step = i * 0.05f;
                        actuators->moveValve(valve, step);
                        Thread::sleep(500);
                    }
                }
                else
                {
                    std::cout << "\nUnknown Command: " << valveCmd << std::endl;
                }
            }
            else
            {
                std::cout << "\nValve not found: " << words[1] << std::endl;
            }
        }
    }

    return 0;
}
