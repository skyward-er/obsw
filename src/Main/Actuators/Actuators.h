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
#include <Main/CanHandler/CanHandler.h>
#include <Main/Configs/ActuatorsConfig.h>
#include <Main/GpioExpander.h>
#include <Main/Sensors/Sensors.h>
#include <actuators/Servo/Servo.h>
#include <actuators/Servo/ServoWinch.h>
#include <algorithms/SchmittTrigger/SchmittTrigger.h>
#include <common/MavlinkHydra.h>
#include <common/UnlimitedAngle.h>
#include <scheduler/TaskScheduler.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <atomic>

namespace Main
{
class Actuators
    : public Boardcore::InjectableWithDeps<BoardScheduler, CanHandler,
                                           GpioExpander, Sensors>
{
public:
    /**
     * @brief Small struct to hold a servo along with per-servo data.
     */
    struct ServoActuator
    {
        std::unique_ptr<Boardcore::ServoWinch> servo;

        /**
         * Used to trigger servo to rotate CW or CCW or to stay still
         */
        std::unique_ptr<Boardcore::SchmittTrigger> servoTrigger;

        bool enabled = false;  // Whether the servo task should adjust position

        Common::UnlimitedAngle angleData;  // Angle of the servo

        Boardcore::Units::Angle::Radian
            zeroAngle;  // initial angle read by the encoder when at zero
                        // position

        Boardcore::Units::Angle::Radian
            minAngle;  // Minimum angle that the servo will reach
        Boardcore::Units::Angle::Radian
            maxAngle;  // Maximum angle that the servo will reach
        Config::Actuators::ServoDirection direction;  // Direction of the servo

        miosix::FastMutex mutex;

        ServoActuator()
            : zeroAngle(Boardcore::Units::Angle::Radian(0.0f)),
              minAngle(Boardcore::Units::Angle::Radian(0.0f)),
              maxAngle(Boardcore::Units::Angle::Radian(0.0f)),
              direction(Config::Actuators::ServoDirection::CW)
        {
        }
    };

    Actuators();

    [[nodiscard]] bool start();

    bool isStarted();

    /**
     * @brief Moves the specified servo to the specified angle.
     * @param angle Angle to set, unlimited range.
     * @return `false` if the servo is invalid, `true` otherwise.
     */
    bool setPrfServoAngle(ServosList servoId,
                          Boardcore::Units::Angle::Radian angle);

    bool wigglePrfServo(ServosList servoId);

    void setPrfServoZero();

    void enablePrfServo(ServosList servoId);
    void disablePrfServo(ServosList servoId);

    void setAbkPosition(float position);
    void wiggleServo(ServosList servo);
    float getServoPosition(ServosList servo);

    void wiggleCanServo(ServosList servo);

    void camOn();
    void camOff();
    bool getCamState();

    void expulsionOn();
    void expulsionOff();
    bool getExpulsionState();

    void releaserOn();
    void releaserOff();
    bool getReleaserState();

    void setBuzzerOff();
    void setBuzzerArmed();
    void setBuzzerLand();

    void setStatusOff();
    void setStatusErr();
    void setStatusOk();

private:
    void unsafeSetServoPosition(Boardcore::Servo* servo, float position);

    void updateServoState(ServosList servoId,
                          Boardcore::Units::Angle::Radian encoderAngle);

    ServoActuator* getServoActuator(ServosList servoId);
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
    std::unique_ptr<Boardcore::PWM> buzzer;

    ServoActuator leftServo;
    ServoActuator rightServo;

    std::atomic<uint32_t> buzzerCounter{0};
    std::atomic<uint32_t> buzzerOverflow{0};

    std::atomic<uint32_t> statusCounter{0};
    std::atomic<uint32_t> statusOverflow{0};

    Boardcore::MCP23S17* expander = nullptr;
};

}  // namespace Main
