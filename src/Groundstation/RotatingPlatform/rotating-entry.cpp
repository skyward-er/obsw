/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Nicolò Caruso
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

#include <Groundstation/Automated/Leds/Leds.h>
#include <Groundstation/LyraGS/Buses.h>
#include <Groundstation/RotatingPlatform/Actuators/Actuators.h>
#include <Groundstation/RotatingPlatform/Actuators/ActuatorsConfig.h>
#include <Groundstation/RotatingPlatform/PinHandler/PinHandler.h>
#include <common/Events.h>
#include <drivers/timer/PWM.h>
#include <drivers/timer/TimerUtils.h>
#include <interfaces-impl/hwmapping.h>

#include <functional>

using namespace Boardcore;
using namespace std;
using namespace std::placeholders;
using namespace Common;

using namespace Antennas;
using namespace LyraGS;
using namespace Boardcore;
using namespace miosix;
using namespace RotatingPlatform;

// Flags indicating if the multipliers have been set
static constexpr float MULTIPLIER_X = 2.2f;  // Multiplier for the stepper X
static constexpr uint8_t ARM_SWITCH_THRESHOLD    = 1;
static constexpr uint8_t FOLLOW_SWITCH_THRESHOLD = 1;

void idleLoop()
{
    while (1)
    {
        Thread::wait();
    }
}

/**
 * @brief Blinking RED led at 5Hz
 */
void errorLoop()
{
    while (1)
    {
        led3On();  //< Turn on RED led (CU)
        Thread::sleep(100);
        led3Off();
        Thread::sleep(100);
    }
}

/**
 * @brief Entrypoint for the rotational platform. To use it, is needed the
 * command box of ARP. The arm switch arms the actuators while the follow one
 * activates the rotation with constant speed (except for a linearly accelerated
 * start and decelerated end)
 *
 * @note The rotation is imposed by a configuration parameter given by
 * constructor or, in this case, by the ActuatorsData stepper config.
 *
 * @attention the LED color code is the following: `RED` always `fast blink`;
 * `YELLOW` `fast blink`: disarmed, `Y-on`: armed; `BLUE` `fast blink`: not
 * rotating, `B-on`: rotating
 */
int main()
{
    DependencyManager manager;
    PrintLogger logger            = Logging::getLogger("rotation_test");
    TaskScheduler* scheduler_high = new TaskScheduler();
    TaskScheduler* scheduler_low  = new TaskScheduler(0);

    Buses* buses                           = new Buses();
    RotatingPlatform::Actuators* actuators = new RotatingPlatform::Actuators();
    Antennas::Leds* leds                   = new Antennas::Leds(scheduler_low);

    bool ok = true;

    ok &= manager.insert(buses);
    ok &= manager.insert(leds);
    ok &= manager.insert(actuators);

    if (!ok)
        errorLoop();

    if (!manager.inject())
    {
        std::cout << "[error] Failed to inject the dependencies!" << std::endl;
        errorLoop();
    }

    if (!Logger::getInstance().start())
    {
        std::cout << "ERROR: Failed to start Logger" << std::endl;
        ok = false;
    }

    if (!scheduler_high->start())
    {
        std::cout << "[error] Failed to start scheduler_high!" << std::endl;
        ok = false;
    }

    if (!scheduler_low->start())
    {
        std::cout << "[error] Failed to start scheduler_low!" << std::endl;
        ok = false;
    }

    // Fast blink to make the operator aware that this is not an ARP
    // entrypoint (lyra-gs-entry)
    leds->setFastBlink(LedColor::RED);
    leds->setFastBlink(LedColor::YELLOW);
    leds->setFastBlink(LedColor::BLUE);

    idleLoop();
}