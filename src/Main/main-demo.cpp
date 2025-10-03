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

#include <Main/Configs/ActuatorsConfig.h>
#include <actuators/Servo/Servo.h>
#include <common/CanConfig.h>
#include <common/MavlinkOrion.h>
#include <drivers/canbus/CanDriver/CanDriver.h>
#include <drivers/canbus/CanProtocol/CanProtocol.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <utils/TimeUtils.h>

#include <bitset>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

using namespace std::chrono;
using namespace miosix;
using namespace Boardcore;
using namespace Main;
using namespace Common;

struct CanValveController
{
    CanValveController()
        : driver{CAN1, CanConfig::CONFIG, CanConfig::BIT_TIMING},
          protocol{&driver, [](const Canbus::CanMessage& msg) {},
                   miosix::MAIN_PRIORITY}
    {
    }

    enum ValveIndex : uint8_t
    {
        OX_VENTING,
        NITROGEN,
        N2_QUENCHING,
        MAIN,
    };

    bool start()
    {
        if (!driver.init(Common::CanConfig::CAN_SYNC_TIMEOUT))
        {
            std::cerr << "Failed to initialize CanbusDriver" << std::endl;
            return false;
        }

        if (!protocol.start())
        {
            std::cerr << "Failed to start CanProtocol" << std::endl;
            return false;
        }

        started = true;
        return true;
    }

    uint8_t mapServo(ValveIndex valve)
    {
        switch (valve)
        {
            case OX_VENTING:
                return ServosList::OX_VENTING_VALVE;
            case NITROGEN:
                return ServosList::NITROGEN_VALVE;
            case N2_QUENCHING:
                return ServosList::N2_QUENCHING_VALVE;
            case MAIN:
            default:
                return ServosList::MAIN_VALVE;
        }
    }

    void openValve(ValveIndex valve)
    {
        protocol.enqueueData(
            static_cast<uint8_t>(CanConfig::Priority::CRITICAL),
            static_cast<uint8_t>(CanConfig::PrimaryType::COMMAND),
            static_cast<uint8_t>(CanConfig::Board::MAIN),
            static_cast<uint8_t>(CanConfig::Board::BROADCAST),
            static_cast<uint8_t>(mapServo(valve)),
            ServoCommand{(unsigned long long)nsToUs(miosix::getTime()),
                         UINT32_MAX});
    }

    void closeValve(ValveIndex valve)
    {
        // Closing a valve means opening it for 0s
        protocol.enqueueData(
            static_cast<uint8_t>(CanConfig::Priority::CRITICAL),
            static_cast<uint8_t>(CanConfig::PrimaryType::COMMAND),
            static_cast<uint8_t>(CanConfig::Board::MAIN),
            static_cast<uint8_t>(CanConfig::Board::BROADCAST),
            static_cast<uint8_t>(mapServo(valve)),
            ServoCommand{(unsigned long long)nsToUs(miosix::getTime()), 0});
    }

    bool started = false;

    Canbus::CanbusDriver driver;
    Canbus::CanProtocol protocol;
};

int main()
{
    miosix::ledOff();
    std::cout << "Main Demo Entrypoint" << std::endl;

    auto servoAbk = std::make_unique<Servo>(
        MIOSIX_AIRBRAKES_TIM, TimerUtils::Channel::MIOSIX_AIRBRAKES_CHANNEL,
        Config::Actuators::ABK_MIN_PULSE, Config::Actuators::ABK_MAX_PULSE);

    servoAbk->enable();
    servoAbk->setPosition(0.0f);
    std::this_thread::sleep_for(1s);

    // Status led indicator: airbrakes active
    miosix::led1On();

    auto valveController = std::make_unique<CanValveController>();
    if (!valveController->start())
    {
        std::cerr << "Failed to start CanValveController, valves won't be used "
                     "in the demo"
                  << std::endl;
        valveController = nullptr;
    }

    // Status led indicator: valves active
    miosix::led2On();

    // Status led indicator: demo initialized
    miosix::led4On();
    std::cout << "Demo init ok!" << std::endl;

    // Move airbrakes from start to end in the given time
    auto animateAbk =
        [&servoAbk](float start, float end, std::chrono::milliseconds time)
    {
        using fseconds = std::chrono::duration<float>;  // float seconds

        // Nothing to move
        if (start == end)
            return;

        float step     = (end - start) / fseconds{time}.count();
        float position = start;
        auto startTime = steady_clock::now();
        auto endTime   = startTime + time;
        auto now       = startTime;

        std::cout << "Animating ABK from " << start << " to " << end << " in "
                  << time.count() << "ms" << std::endl;

        while (now < endTime)
        {
            servoAbk->setPosition(position);
            now          = steady_clock::now();
            auto elapsed = fseconds{now - startTime};
            startTime    = now;

            position += step * elapsed.count();

            std::this_thread::sleep_for(10ms);
        }
        servoAbk->setPosition(end);
    };

    // Initial open/close cycles with increasing speed
    for (int i = 0; i < 3; i++)
    {
        std::cout << "ABK open/close cycle " << i << " (" << i * 200 << " ms)"
                  << std::endl;
        // Fully open airbrakes
        animateAbk(0.0f, 1.0f, 200ms * i);
        std::this_thread::sleep_for(1s);
        // Fully close airbrakes
        animateAbk(1.0f, 0.0f, 200ms * i);
        std::this_thread::sleep_for(2s);
    }

    float position             = 0.0f;
    auto lastAbkAction         = steady_clock::now();
    std::bitset<4> valveStatus = {0b0000};
    auto lastValveAction       = steady_clock::now();

    while (true)
    {
        if (servoAbk && steady_clock::now() - lastAbkAction > 3s)
        {
            // Get a random position that is different enough from the current
            // one
            float newPosition = 0.0f;
            do
                newPosition = static_cast<float>(std::rand()) / RAND_MAX;
            while (std::abs(position - newPosition) < 0.5f);

            std::cout << "ABK position change: " << position << " -> "
                      << newPosition << std::endl;

            constexpr int minAnimTimeMs = 100;
            constexpr int maxAnimTimeMs = 300;
            auto animTime               = std::chrono::milliseconds(
                minAnimTimeMs +
                std::rand() % (maxAnimTimeMs - minAnimTimeMs + 1));
            animateAbk(position, newPosition, animTime);

            position      = newPosition;
            lastAbkAction = steady_clock::now();
        }

        if (valveController && steady_clock::now() - lastValveAction > 10s)
        {
            using ValveIndex = CanValveController::ValveIndex;

            // Open/close a random valve
            int valve = std::rand() % 4;
            valveStatus.flip(valve);

            std::cout << "Valve status: " << valveStatus << std::endl;

            if (valveStatus.test(valve))
                valveController->openValve(static_cast<ValveIndex>(valve));
            else
                valveController->closeValve(static_cast<ValveIndex>(valve));

            lastValveAction = steady_clock::now();
        }

        {
            using led = miosix::gpios::statusLed;
            led::value() ? led::low() : led::high();
            std::this_thread::sleep_for(1s);
        }
    }

    return 0;
}
