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

#include <actuators/Servo/ServoWinch.h>
#include <algorithms/SchmittTrigger/SchmittTrigger.h>
#include <common/MavlinkOrion.h>
#include <common/UnlimitedAngle.h>
#include <units/Angle.h>
#include <utils/DependencyManager/DependencyManager.h>

using namespace Boardcore::Units::Angle;

namespace Parafoil
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
        std::unique_ptr<Boardcore::ServoWinch> servo;

        /**
         * Used to trigger servo to rotate CW or CCW or to stay still
         */
        std::unique_ptr<Boardcore::SchmittTrigger> servoTrigger;

        /**
         * Used to encode the angle data reading from the encoders
         */
        Common::UnlimitedAngle angleData;

        /**
         * Used to set the servo velocity when the Schmitt trigger output is
         * HIGH
         */
        float highServoVelocity;

        /**
         * Used to set the servo velocity when the Schmitt trigger output is LOW
         */
        float lowServoVelocity;

        /**
         * Used to set the servo velocity when the Schmitt trigger output is
         * STOP
         */

        float stopServoVelocity;

        /**
         * The angle to which the wiggle procedure
         */
        Radian wiggleAngle;

        miosix::FastMutex mutex;

        ServoActuator() : wiggleAngle{0_rad} {}
    };

    Actuators();

    [[nodiscard]] bool start();

    bool isStarted();

    /**
     * @brief Moves the specified servo to the specified position.
     * @param percentage velocity to set in the range [0-1] (0.5 means stop,
     * <0.5 means rotate CCW, >0.5 meas rotate CW)
     * @return `false` if the servo is invalid, `true` otherwise.
     */
    bool setServoVelocity(ServosList servoId, float velocity);

    /**
     * @brief Moves the specified servo to the specified angle.
     * @param angle Angle to set, unlimited range.
     * @return `false` if the servo is invalid, `true` otherwise.
     */
    bool setServoAngle(ServosList servoId, Radian angle);

    /**
     * @brief Returns the current velocity of the specified servo in range
     * [0-1], or a negative value (-1) if the servo is invalid.
     */
    float getServoVelocity(ServosList servoId);

    /**
     * @brief Returns the current angle of the specified servo
     */
    Radian getServoAngle(ServosList servoId);

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

    /**
     * @brief updates the unlimited angle estimator and the schmitt trigger with
     * a new angle reading (likely from an external encoder, like the AS5047D)
     * for the given servo
     */
    void updateServoState(ServosList servoId, Radian angle);

    /**
     * This method checks if the servo is moving
     * @param servoId the servo to check
     * @returns true if the servo is moving, false otherwise
     */
    bool servoIsMoving(ServosList servoId);

private:
    ServoActuator* getServoActuator(ServosList servoId);

public:
    void setBuzzerOff();
    void setBuzzerLanded();
    void setBuzzerArmed();
    void setBuzzerNoseconeDetached();

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

    uint32_t buzzerSequence[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    std::atomic<uint32_t> buzzerSequenceCounter{0};
    std::atomic<uint32_t> buzzerCounter{0};
    std::atomic<uint32_t> buzzerThreshold{0};

    std::atomic<uint32_t> statusLedCounter{0};
    std::atomic<uint32_t> statusLedThreshold{0};

    std::atomic<bool> started{false};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Actuators");
};

}  // namespace Parafoil
