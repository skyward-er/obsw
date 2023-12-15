/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <Parafoil/Actuators/Actuators.h>
#include <Parafoil/AltitudeTrigger/AltitudeTrigger.h>
#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Buses.h>
#include <Parafoil/Configs/SensorsConfig.h>
#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/PinHandler/PinHandler.h>
#include <Parafoil/Radio/Radio.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <Parafoil/StateMachines/WingController/WingController.h>
#include <Parafoil/TMRepository/TMRepository.h>
#include <Parafoil/WindEstimationScheme/WindEstimation.h>
#include <Parafoil/Wing/AutomaticWingAlgorithm.h>
#include <Parafoil/Wing/FileWingAlgorithm.h>
#include <common/Events.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/utils/EventSniffer.h>
#include <miosix.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace Parafoil;
using namespace Common;
using namespace Parafoil::SensorsConfig;

int main()
{
    Thread::sleep(500);  // wait 500 ms to separate the mcu startup and the ubx
                         // serial communication

    bool initResult = true;

    PrintLogger logger     = Logging::getLogger("main");
    ModuleManager& modules = ModuleManager::getInstance();

    // Starting singleton
    {
        if (!BoardScheduler::getInstance().getScheduler().start())
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing the BoardScheduler");
        }
        if (!EventBroker::getInstance().start())
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing WingController module");
        }
        if (!Logger::getInstance().start())
        {
            initResult = false;
            LOG_ERR(logger, "Error initializing the Logger");
        }
    }

    // Initialize the modules
    {
        if (!modules.insert<Actuators>(new Actuators()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the Actuators module");
        }

        if (!modules.insert<AltitudeTrigger>(new AltitudeTrigger()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the AltitudeTrigger module");
        }

        if (!modules.insert<Buses>(new Buses()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the Buses module");
        }

        if (!modules.insert<FlightModeManager>(new FlightModeManager()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the FlightModeManager module");
        }

        if (!modules.insert<NASController>(new NASController()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the NASController module");
        }

        if (!modules.insert<PinHandler>(new PinHandler()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the PinHandler module");
        }

        if (!modules.insert<Radio>(new Radio()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the Radio module");
        }

        if (!modules.insert<Sensors>(new Sensors()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the Sensors module");
        }

        if (!modules.insert<TMRepository>(new TMRepository()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the TMRepository module");
        }

        if (!modules.insert<WindEstimation>(new WindEstimation()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the WindEstimation module");
        }

        if (!modules.insert<WingController>(new WingController()))
        {
            initResult = false;
            LOG_ERR(logger, "Error inserting the WingController module");
        }
    }

    // Start the modules
    {
        if (!modules.get<Actuators>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error starting the Actuators module");
        }

        if (!modules.get<AltitudeTrigger>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error starting the AltitudeTrigger module");
        }

        if (!modules.get<Buses>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error starting the Buses module");
        }

        if (!modules.get<FlightModeManager>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error starting the FlightModeManager module");
        }

        if (!modules.get<NASController>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error starting the NASController module");
        }

        if (!modules.get<PinHandler>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error starting the PinHandler module");
        }

        if (!modules.get<Radio>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error starting the Radio module");
        }

        if (!modules.get<Sensors>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error starting the Sensors module");
        }

        if (!modules.get<TMRepository>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error starting the TMRepository module");
        }

        if (!modules.get<WindEstimation>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error starting the WindEstimation module");
        }

        if (!modules.get<WingController>()->startModule())
        {
            initResult = false;
            LOG_ERR(logger, "Error starting the WingController module");
        }
    }
    WingConfig::WingConfigStruct f;
    modules.get<WingController>()->addAlgorithm(WingConfig::SELECTED_ALGORITHM);
    Logger::getInstance().log(f);  // logs the config file
    // If all is correctly set up i publish the init ok
    if (initResult)
        EventBroker::getInstance().post(FMM_INIT_OK, TOPIC_FMM);
    else
        EventBroker::getInstance().post(FMM_INIT_ERROR, TOPIC_FMM);

    // Log all events
    EventSniffer sniffer(
        EventBroker::getInstance(), TOPICS_LIST,
        [](uint8_t event, uint8_t topic)
        {
            EventData ev{TimestampTimer::getTimestamp(), event, topic};
            Logger::getInstance().log(ev);
        });

    GpioPin sckPin  = GpioPin(GPIOG_BASE, 13);
    GpioPin misoPin = GpioPin(GPIOG_BASE, 12);

    // Setup gpio pins
    sckPin.mode(Mode::ALTERNATE);
    sckPin.alternateFunction(5);
    misoPin.mode(Mode::ALTERNATE);
    misoPin.alternateFunction(5);

    SPIBus spiBus(SPI6);
    HX711 loadCell{spiBus, sckPin};

    loadCell.setOffset(LOAD_CELL1_OFFSET);
    loadCell.setScale(LOAD_CELL1_SCALE);

    // Periodically statistics
    while (true)
    {
        for (int i = 0; i < 80; i++)
        {
            loadCell.sample();

            Logger::getInstance().log(loadCell.getLastSample());
            Thread::sleep(1000.0 / 80.0);
        }
        Logger::getInstance().log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        Logger::getInstance().logStats();
        modules.get<Radio>()->logStatus();
        StackLogger::getInstance().log();
    }
}
