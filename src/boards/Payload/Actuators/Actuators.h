/* Copyright (c) 2024 Skyward Experimental Rocketry
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

#include <actuators/Servo/Servo.h>
#include <common/Mavlink.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Payload
{
class BoardScheduler;

class Actuators : public Boardcore::InjectableWithDeps<BoardScheduler>
{
public:
    /**
     * @brief A small struct to hold a servo along with per-servo data.
     */
    struct ServoActuator
    {
        std::unique_ptr<Boardcore::Servo> servo;
        float fullRangeAngle;  ///< The full range of the servo [degrees]
        miosix::FastMutex mutex;
    };

    Actuators();

    [[nodiscard]] bool start();

    bool isStarted();

    /**
     * @brief Moves the specified servo to the specified position.
     * @param percentage Position to set in the range [0-1]
     * @return `false` if the servo is invalid, `true` otherwise.
     */
    bool setServoPosition(ServosList servoId, float position);

    /**
     * @brief Moves the specified servo to the specified angle.
     * @param angle Angle to set in the range [0-180] [degrees]
     * @return `false` if the servo is invalid, `true` otherwise.
     */
    bool setServoAngle(ServosList servoId, float angle);

    /**
     * @brief Returns the current position of the specified servo in range
     * [0-1], or a negative value (-1) if the servo is invalid.
     */
    float getServoPosition(ServosList servoId);

    /**
     * @brief Returns the current angle of the specified servo in range [0-180],
     * or a negative value (-1) if the servo is invalid.
     */
    float getServoAngle(ServosList servoId);

    /**
     * @brief Wiggles the specified servo. This function is blocking, it
     * sleeps the caller thread between the two movements.
     * @return `false` if the servo is invalid, `true` otherwise.
     */
    bool wiggleServo(ServosList servoId);

    /**
     * @brief Disables the specified servo.
     * @return `false` if the servo is invalid, `true` otherwise.
     */
    bool disableServo(ServosList servoId);

private:
    ServoActuator* getServoActuator(ServosList servoId);

public:
    void setBuzzerOff();
    void setBuzzerOnLand();
    void setBuzzerArmed();

    void setStatusOff();
    void setStatusOk();
    void setStatusError();

    void cameraOn();
    void cameraOff();

    void cuttersOn();
    void cuttersOff();

private:
    void statusOn();
    void statusOff();

    void buzzerOn();
    void buzzerOff();

    // Periodic task functions that update the buzzer and status LED
    void updateBuzzer();
    void updateStatusLed();

    ServoActuator leftServo;
    ServoActuator rightServo;
    std::unique_ptr<Boardcore::PWM> buzzer;

    std::atomic<uint32_t> buzzerCounter{0};
    std::atomic<uint32_t> buzzerThreshold{0};

    std::atomic<uint32_t> statusLedCounter{0};
    std::atomic<uint32_t> statusLedThreshold{0};

    std::atomic<bool> started{false};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Actuators");
};

}  // namespace Payload
