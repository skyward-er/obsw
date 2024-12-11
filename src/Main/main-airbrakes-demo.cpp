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
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>

#include <chrono>
#include <iomanip>
#include <iostream>

using namespace std::chrono;
using namespace miosix;
using namespace Boardcore;
using namespace Main;

int main()
{
    ledOff();
    std::cout << "Airbrakes Demo Entrypoint" << std::endl;

    Servo servoAbk(
        MIOSIX_AIRBRAKES_TIM, TimerUtils::Channel::MIOSIX_AIRBRAKES_CHANNEL,
        Config::Actuators::ABK_MIN_PULSE, Config::Actuators::ABK_MAX_PULSE);

    servoAbk.enable();
    servoAbk.setPosition(0.0f);
    Thread::sleep(1000);

    // Status led indicators
    // led4: Everything ok
    led4On();
    std::cout << "Initialization ok!" << std::endl;

    using fseconds = std::chrono::duration<float>;  // float seconds

    // Move airbrakes from start to end in the given time
    auto animateAbk =
        [&servoAbk](float start, float end, std::chrono::milliseconds time)
    {
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
            servoAbk.setPosition(position);
            now          = steady_clock::now();
            auto elapsed = fseconds{now - startTime};
            startTime    = now;

            position += step * elapsed.count();

            Thread::sleep(10);
        }
        servoAbk.setPosition(end);
    };

    for (int i = 0; i < 3; i++)
    {
        std::cout << "ABK open/close cycle " << i << " (" << i * 200 << " ms)"
                  << std::endl;
        // Fully open airbrakes
        animateAbk(0.0f, 1.0f, 200ms * i);
        Thread::sleep(1000);
        // Fully close airbrakes
        animateAbk(1.0f, 0.0f, 200ms * i);
        Thread::sleep(2000);
    }

    float position = 0.0f;

    while (true)
    {
        // Get a random position that is different enough from the current one
        float newPosition = 0.0f;
        do
            newPosition = static_cast<float>(std::rand()) / RAND_MAX;
        while (std::abs(position - newPosition) < 0.3f);

        std::cout << "ABK position change: " << position << " -> "
                  << newPosition << std::endl;
        animateAbk(position, newPosition, 300ms);

        position = newPosition;
        Thread::sleep(2000);
    }

    return 0;
}
