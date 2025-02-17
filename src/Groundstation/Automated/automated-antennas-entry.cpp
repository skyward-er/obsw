/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Riccardo Musso, Emilio Corigliano, Niccolò Betto, Federico Lolli
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

#include <Groundstation/Automated/Actuators/Actuators.h>
#include <Groundstation/Automated/Buses.h>
#include <Groundstation/Automated/Hub.h>
#include <Groundstation/Automated/Leds/Leds.h>
#include <Groundstation/Automated/Ports/Ethernet.h>
#include <Groundstation/Automated/Radio/Radio.h>
#include <Groundstation/Automated/SMA/SMA.h>
#include <Groundstation/Automated/Sensors/Sensors.h>
#include <Groundstation/Common/Ports/Serial.h>
#include <Groundstation/LyraGS/BoardStatus.h>
#include <common/Events.h>
#include <diagnostic/PrintLogger.h>
#include <events/EventBroker.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>
#include <utils/ButtonHandler/ButtonHandler.h>

#include <thread>
#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Groundstation;
using namespace Antennas;
using namespace Common;
using namespace Boardcore;
using namespace miosix;

GpioPin button = GpioPin(GPIOG_BASE, 10);  ///< Emergency stop button

/**
 * @brief Automated Antennas (SkyLink) entrypoint.
 * The entrypoint performs the following operations:
 * - Initializes software modules
 *   -> Green LED is turned on when done
 * - Waits for the rocket to be powered on and acquire a GPS fix
 *   -> Yellow LED is turned on when done
 * - Waits for the antenna to acquire a GPS fix
 *   -> Orange LED is turned on when done
 * - Initializes the follower
 * - Starts the follower task
 */
int main()
{
    ModuleManager& modules = ModuleManager::getInstance();
    PrintLogger logger     = Logging::getLogger("automated_antennas");
    bool ok                = true;

    button.mode(Mode::INPUT_PULL_UP);
    // ButtonHandler
    ButtonHandler::getInstance().registerButtonCallback(
        button,
        [&](ButtonEvent bEvent)
        {
            if (bEvent == ButtonEvent::LONG_PRESS ||
                bEvent == ButtonEvent::VERY_LONG_PRESS)
            {
                ModuleManager::getInstance()
                    .get<Actuators>()
                    ->IRQemergencyStop();
            }
        });
    ButtonHandler::getInstance().start();

    TaskScheduler* scheduler_low  = new TaskScheduler(0);
    TaskScheduler* scheduler_high = new TaskScheduler();
    Leds* leds                    = new Leds(scheduler_low);
    Hub* hub                      = new Hub();
    Buses* buses                  = new Buses();
    Serial* serial                = new Serial();
    RadioMain* radio_main         = new RadioMain();
    BoardStatus* board_status     = new BoardStatus();
    Actuators* actuators          = new Actuators();
    Sensors* sensors              = new Sensors();
    SMA* sma                      = new SMA(scheduler_high);
    Ethernet* ethernet            = new Ethernet();

    // Inserting Modules
    {  // TODO remove this scope (improve readability)
        ok &= modules.insert(sma);
        ok &= modules.insert<HubBase>(hub);
        ok &= modules.insert(buses);
        ok &= modules.insert(serial);
        ok &= modules.insert(radio_main);
        ok &= modules.insert(board_status);
        ok &= modules.insert(actuators);
        ok &= modules.insert(sensors);
        ok &= modules.insert(ethernet);
        ok &= modules.insert(leds);

        // If insertion failed, stop right here
        if (!ok)
        {
            LOG_ERR(logger, "Failed to insert all modules!\n");
            leds->endlessBlink(LedColor::RED, LED_BLINK_FAST_PERIOD_MS);
        }
        else
        {
            LOG_DEBUG(logger, "All modules inserted successfully!\n");
        }
    }

    // Starting Modules
    bool init_fatal = false;
    bool init_error = false;

#ifndef NO_SD_LOGGING
    if (!Logger::getInstance().start())
    {
        LOG_ERR(logger, "ERROR: Failed to start Logger\n");
        init_error = true;
    }
#endif
    if (!scheduler_low->start())
    {
        LOG_ERR(logger, "ERROR: Failed to start Scheduler Low\n");
        init_error = true;
    }
    if (!scheduler_high->start())
    {
        LOG_ERR(logger, "FATAL: Failed to start Scheduler High\n");
        init_fatal = true;
    }
    if (!serial->start())
    {
        LOG_ERR(logger, "ERROR: Failed to start Serial\n");
        init_error = true;
    }
    if (!leds->start())
    {
        LOG_ERR(logger, "ERROR: Failed to start Leds\n");
        init_error = true;
    }
    if (!radio_main->start())
    {
        LOG_ERR(logger, "FATAL: Failed to start Main Radio\n");
        init_fatal = true;
    }
    if (!ethernet->start())
    {
        LOG_ERR(logger, "ERROR: Failed to start Ethernet\n");
        init_error = true;
    }
    if (!board_status->start())
    {
        LOG_ERR(logger, "ERROR: Failed to start Board Status\n");
        init_error = true;
    }
    if (!sensors->start())
    {
        LOG_ERR(logger, "ERROR: Failed to start Sensors\n");
        init_error = true;
    }
    if (!sma->start())
    {
        LOG_ERR(logger, "FATAL: Failed to start SMA\n");
        init_fatal = true;
    }
    actuators->start();

    LOG_INFO(logger, "Modules setup successful");

    if (board_status->isMainRadioPresent())
        LOG_DEBUG(logger, "Main radio is present\n");

    // If init fatal and sma not started, blink red endlessly
    if (init_fatal)
    {
        leds->endlessBlink(LedColor::RED, LED_BLINK_FAST_PERIOD_MS);
    }  // If another module is in error
    else if (init_error)
    {
        EventBroker::getInstance().post(ARP_INIT_ERROR, TOPIC_ARP);
    }  // If all modules are ok
    else
    {
        LOG_INFO(logger, "Starting ARP");
        EventBroker::getInstance().post(ARP_INIT_OK, TOPIC_ARP);
    }

    while (true)
        Thread::wait();
    return 0;
}
