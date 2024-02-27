/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <actuators/Servo/Servo.h>
#include <common/Mavlink.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>

#include <memory>
#include <utils/ModuleManager/ModuleManager.hpp>

namespace RIGv2
{

class Actuators : public Boardcore::Module
{
private:
    struct ServoInfo
    {
        std::unique_ptr<Boardcore::Servo> servo;
        // How much time to stay open
        uint64_t openingTime = 100000;  // Default 100s [ms]
        // What angle is the maximum
        float maxAperture = 1.0;
        // Should this servo be reversed?
        bool flipped = false;

        // What event to fire while opening?
        uint8_t openingEvent = 0;
        // What event to fire while closing?
        uint8_t closingEvent = 0;

        // Timestamp of when the servo should close, 0 if closed
        long long closeTs = 0;
        // Timestamp of when the servo was opened
        long long openedTs = 0;

        void openServo(uint64_t time);
        void closeServo();
        void unsafeSetServoPosition(float position);
        float getServoPosition();
    };

public:
    Actuators(Boardcore::TaskScheduler &scheduler);

    [[nodiscard]] bool start();

    bool wiggleServo(ServosList servo);
    bool toggleServo(ServosList servo);
    bool closeServo(ServosList servo);
    bool openServoAtomic(ServosList servo, uint64_t time);
    bool setMaxAperture(ServosList servo, float aperture);
    bool setOpeningTime(ServosList servo, uint64_t time);
    bool isServoOpen(ServosList servo);

private:
    ServoInfo *getServo(ServosList servo);

    void unsafeSetServoPosition(uint8_t idx, float position);
    void updatePositionsTask();

    Boardcore::Logger &sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("actuators");

    Boardcore::TaskScheduler &scheduler;

    size_t updatePositionTaskId = 0;
    miosix::FastMutex infosMutex;
    ServoInfo infos[10] = {};
};

}  // namespace RIGv2