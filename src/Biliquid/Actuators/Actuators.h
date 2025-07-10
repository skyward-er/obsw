/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#pragma once

#include <RIGv2/BoardScheduler.h>
#include <RIGv2/CanHandler/CanHandler.h>
#include <RIGv2/Registry/Registry.h>
#include <actuators/Servo/Servo.h>
#include <common/MavlinkOrion.h>
#include <miosix.h>
#include <scheduler/SignaledDeadlineTask.h>
#include <scheduler/TaskScheduler.h>

#include <memory>

namespace Biliquid
{

enum class Valve
{
    MAIN_OX,    ///< Main oxidizer valve
    MAIN_FUEL,  ///< Main fuel valve
};

class Actuators : private Boardcore::SignaledDeadlineTask
{
private:
    // Sentinel value for the valve closed state
    static const TimePoint ValveClosed;

    struct ValveInfo
    {
        struct ValveConfig
        {
            Valve id;  ///< Valve ID

            float limit       = 1.0;    ///< Movement range limit
            bool flipped      = false;  ///< Whether the servo is flipped
            float maxAperture = 1.0;    // Max aperture

            uint8_t openingEvent = 0;  ///< Event to fire after opening
            uint8_t closingEvent = 0;  ///< Event to fire after closing
        };

        ValveInfo(std::unique_ptr<Boardcore::Servo> servo,
                  const ValveConfig& config)
            : servo(std::move(servo)), config(config)
        {
        }

        std::unique_ptr<Boardcore::Servo> servo;
        ValveConfig config;

        float currentPosition = 0.0f;  ///< Current position in range [0, 1]
        enum class Direction
        {
            CLOSE,
            OPEN,
        } direction = Direction::CLOSE;  ///< Direction of the last valve move

        // Time when to backstep the valve to avoid straining the servo
        TimePoint backstepTs = ValveClosed;

        void open(float position);
        void close();

        void move();
        void backstep();

        float scalePosition(float position);
        float toDegrees(float position);
    };

public:
    Actuators();

    [[nodiscard]] bool start();

    bool isStarted();

    bool openValve(Valve valveId, float position);

    bool closeValve(Valve valveId);

    void closeAll();

private:
    ValveInfo* getValve(Valve valveId);

    TimePoint nextTaskDeadline() override;
    void task() override;

    std::atomic<bool> started{false};

    miosix::FastMutex valveMutex;
    std::array<ValveInfo, 2> valves;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("actuators");
};

}  // namespace Biliquid
