/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Niccolò Betto
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
#include <Groundstation/Automated/Radio/Radio.h>
#include <Groundstation/Automated/Sensors/Sensors.h>
#include <Groundstation/Common/Ports/Serial.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/interrupt/external_interrupts.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>
#include <utils/ButtonHandler/ButtonHandler.h>

#define STEPPER_SPEED 0.25
#define TEST_WAIT 5000

#define START_MODULE(name, lambda)                                  \
    do                                                              \
    {                                                               \
        std::function<bool()> _fun = lambda;                        \
        if (!_fun())                                                \
        {                                                           \
            LOG_ERR(logger, "Failed to start module " name);        \
            errorLoop();                                            \
        }                                                           \
        else                                                        \
        {                                                           \
            LOG_DEBUG(logger, "Successfully started module " name); \
        }                                                           \
    } while (0)

using namespace Groundstation;
using namespace Antennas;
using namespace Boardcore;
using namespace miosix;

GpioPin button = GpioPin(GPIOG_BASE, 10);  ///< Emergency stop button

void __attribute__((used)) EXTI10_IRQHandlerImpl()
{
    ModuleManager::getInstance().get<Actuators>()->IRQemergencyStop();
}

void ledWaitLoop(int ms)
{
    int waited = 0;
    while (waited < ms)
    {
        led2On();
        Thread::sleep(100);
        led2Off();
        Thread::sleep(100);
        waited += 200;
    }
}

void errorLoop()
{
    while (1)
    {
        userLed4::high();
        Thread::sleep(100);
        userLed4::low();
        Thread::sleep(100);
    }
}

void test1(Actuators *actuators)
{
    PrintLogger logger = PrintLogger{Logging::getLogger("test-actuators")};
    LOG_INFO(logger, "Executing Test 1");

    // theta1 : HORIZONTAL
    // theta2 : VERTICAL
    LOG_DEBUG(logger, "Setting speed to speed\n");
    actuators->setSpeed(StepperList::STEPPER_X, STEPPER_SPEED);
    actuators->setSpeed(StepperList::STEPPER_Y, STEPPER_SPEED);

    LOG_DEBUG(logger, "Moving 90deg horizontally\n");
    actuators->moveDeg(StepperList::STEPPER_X, 90);
    ledWaitLoop(TEST_WAIT);

    LOG_DEBUG(logger, "Moving back to the initial horizontal position\n");
    actuators->moveDeg(StepperList::STEPPER_X, -90);
    ledWaitLoop(TEST_WAIT);

    LOG_DEBUG(logger, "Moving -90deg horizontally\n");
    actuators->moveDeg(StepperList::STEPPER_X, -90);
    ledWaitLoop(TEST_WAIT);

    LOG_DEBUG(logger, "Moving back to the initial horizontal position\n");
    actuators->moveDeg(StepperList::STEPPER_X, 90);
    ledWaitLoop(TEST_WAIT);

    LOG_INFO(logger, "Test 1 completed\n");
}

void test2(Actuators *actuators)
{
    PrintLogger logger = PrintLogger{Logging::getLogger("test-actuators")};
    LOG_INFO(logger, "Executing Test 2");

    actuators->setSpeed(StepperList::STEPPER_X, STEPPER_SPEED);
    actuators->setSpeed(StepperList::STEPPER_Y, STEPPER_SPEED);
    actuators->moveDeg(StepperList::STEPPER_Y, 22.5);
    ledWaitLoop(TEST_WAIT);

    actuators->moveDeg(StepperList::STEPPER_X, 90);
    ledWaitLoop(TEST_WAIT);

    actuators->moveDeg(StepperList::STEPPER_X, -90);
    ledWaitLoop(TEST_WAIT);

    actuators->moveDeg(StepperList::STEPPER_X, -90);
    ledWaitLoop(TEST_WAIT);

    actuators->moveDeg(StepperList::STEPPER_X, 90);
    ledWaitLoop(TEST_WAIT);

    // Return to (0,0)
    actuators->moveDeg(StepperList::STEPPER_Y, -22.5);
    ledWaitLoop(TEST_WAIT);

    LOG_INFO(logger, "Test 2 completed\n");
}

void test3(Actuators *actuators)
{
    actuators->setSpeed(StepperList::STEPPER_X, STEPPER_SPEED);
    actuators->setSpeed(StepperList::STEPPER_Y, STEPPER_SPEED);

    actuators->moveDeg(StepperList::STEPPER_X, 180);
    ledWaitLoop(TEST_WAIT + 4000);
    actuators->moveDeg(StepperList::STEPPER_X, -180);
    ledWaitLoop(TEST_WAIT + 4000);

    actuators->moveDeg(StepperList::STEPPER_Y, 90);
    ledWaitLoop(TEST_WAIT);
    actuators->moveDeg(StepperList::STEPPER_Y, -90);
    ledWaitLoop(TEST_WAIT);
}

void test6(Actuators *actuators)
{
    PrintLogger logger = PrintLogger{Logging::getLogger("test-actuators")};
    LOG_INFO(logger, "Executing Test 6");

    actuators->setSpeed(StepperList::STEPPER_X, STEPPER_SPEED);
    actuators->setSpeed(StepperList::STEPPER_Y, STEPPER_SPEED);

    actuators->moveDeg(StepperList::STEPPER_Y, 22.5);
    ledWaitLoop(TEST_WAIT);

    actuators->moveDeg(StepperList::STEPPER_Y, -22.5);
    ledWaitLoop(TEST_WAIT);

    actuators->moveDeg(StepperList::STEPPER_Y, 45);
    ledWaitLoop(TEST_WAIT);

    actuators->moveDeg(StepperList::STEPPER_Y, -45);
    ledWaitLoop(TEST_WAIT);

    actuators->moveDeg(StepperList::STEPPER_Y, 65.5);
    ledWaitLoop(TEST_WAIT);

    actuators->moveDeg(StepperList::STEPPER_Y, -65.5);
    ledWaitLoop(TEST_WAIT);

    actuators->moveDeg(StepperList::STEPPER_Y, 90);
    ledWaitLoop(TEST_WAIT);

    actuators->moveDeg(StepperList::STEPPER_Y, -90);
    ledWaitLoop(TEST_WAIT);

    LOG_INFO(logger, "Test 6 completed\n");
}

int main()
{
    ledOff();
    button.mode(Mode::INPUT);
    enableExternalInterrupt(button.getPort(), button.getNumber(),
                            InterruptTrigger::RISING_EDGE);

    Hub *hub             = new Hub();
    Buses *buses         = new Buses();
    Serial *serial       = new Serial();
    Actuators *actuators = new Actuators();
    Sensors *sensors     = new Sensors();

    ModuleManager &modules = ModuleManager::getInstance();
    PrintLogger logger     = PrintLogger{Logging::getLogger("test-actuators")};
    bool ok                = true;

    LOG_INFO(logger, "test-actuators\n");

    // Insert modules
    {
        ok &= modules.insert<HubBase>(hub);
        ok &= modules.insert(buses);
        ok &= modules.insert(serial);
        ok &= modules.insert(actuators);
        ok &= modules.insert(sensors);

        // If insertion failed, stop right here
        if (!ok)
        {
            LOG_ERR(logger, "Failed to insert all modules!\n");
            errorLoop();
        }
        else
        {
            LOG_DEBUG(logger, "All modules inserted successfully!\n");
        }
    }

    // Start modules
    {
#ifndef NO_SD_LOGGING
        START_MODULE("Logger", [&] { return Logger::getInstance().start(); });
#endif
        START_MODULE("Serial", [&] { return serial->start(); });
        START_MODULE("Sensors", [&] { return sensors->start(); });

        actuators->start();
        LOG_DEBUG(logger, "Actuators started successfully!\n");
    }

    // Setup success LED
    led1On();
    LOG_INFO(logger, "Modules setup successful");

    LOG_INFO(logger, "Starting tests\n");

    test1(actuators);
    test2(actuators);
    test3(actuators);
    test6(actuators);

    LOG_INFO(logger, "Tests completed\n");

    led1Off();
    while (1)
    {
        Thread::sleep(1000);
    }
    return 0;
}