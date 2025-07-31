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

#include <Main/BoardScheduler.h>
#include <actuators/Servo/Servo.h>
#include <common/MavlinkLyra.h>
#include <scheduler/TaskScheduler.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <atomic>

namespace Main
{

class CanHandler;

class Actuators
    : public Boardcore::InjectableWithDeps<BoardScheduler, CanHandler>
{
public:
    Actuators();

    [[nodiscard]] bool start();

    bool isStarted();

    void setAbkPosition(float position);
    void openExpulsion();

    void wiggleServo(ServosList servo);
    float getServoPosition(ServosList servo);

    void wiggleCanServo(ServosList servo);

    void camOn();
    void camOff();
    bool getCamState();

    void cutterOn();
    void cutterOff();
    bool getCutterState();

    void setBuzzerOff();
    void setBuzzerArmed();
    void setBuzzerLand();

    void setStatusOff();
    void setStatusErr();
    void setStatusOk();

private:
    void unsafeSetServoPosition(Boardcore::Servo* servo, float position);

    Boardcore::Servo* getServo(ServosList servo);

    void statusOn();
    void statusOff();

    void buzzerOn();
    void buzzerOff();

    void updateBuzzer();
    void updateStatus();

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("actuators");
    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();

    std::atomic<bool> started{false};

    miosix::FastMutex servosMutex;
    std::unique_ptr<Boardcore::Servo> servoAbk;
    std::unique_ptr<Boardcore::Servo> servoExp;
    std::unique_ptr<Boardcore::PWM> buzzer;

    std::atomic<uint32_t> buzzerCounter{0};
    std::atomic<uint32_t> buzzerOverflow{0};

    std::atomic<uint32_t> statusCounter{0};
    std::atomic<uint32_t> statusOverflow{0};
};

}  // namespace Main
