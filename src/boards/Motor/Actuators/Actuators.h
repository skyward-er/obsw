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

#pragma once

#include <Motor/BoardScheduler.h>
#include <Motor/CanHandler/CanHandler.h>
#include <actuators/Servo/Servo.h>
#include <common/Mavlink.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Motor
{

class Actuators
    : public Boardcore::InjectableWithDeps<BoardScheduler, CanHandler>
{
private:
    struct ServoInfo
    {
        std::unique_ptr<Boardcore::Servo> servo;
        // Hard limit of the aperture
        float limit = 1.0;
        // Should this servo be reversed?
        bool flipped = false;
        // Timestamp of when the servo should close, 0 if closed
        long long closeTs = 0;
        // Timestamp of last servo action (open/close)
        long long lastActionTs = 0;

        void openServoWithTime(uint32_t time);
        void closeServo();
        void unsafeSetServoPosition(float position);
        float getServoPosition();
    };

public:
    Actuators();

    [[nodiscard]] bool start();

    bool openServoWithTime(ServosList servo, uint32_t time);
    bool closeServo(ServosList servo);
    bool isServoOpen(ServosList servo);
    float getServoPosition(ServosList servo);

private:
    ServoInfo *getServo(ServosList servo);

    void unsafeSetServoPosition(uint8_t idx, float position);

    void updatePositionsTask();

    Boardcore::Logger &sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("actuators");

    miosix::FastMutex infosMutex;
    ServoInfo infos[2] = {};
};

}  // namespace Motor