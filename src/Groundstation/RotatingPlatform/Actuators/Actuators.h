/* Copyright (c) 2023-2024 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano, Nicolò Caruso
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

#include <ActiveObject.h>
#include <common/MavlinkLyra.h>
#include <diagnostic/PrintLogger.h>
#include <logger/Logger.h>
#include <utils/DependencyManager/DependencyManager.h>

#include "ActuatorsConfig.h"
#include "ActuatorsData.h"
#include "actuators/stepper/StepperPWM.h"

// TIM1_CH1 PA8 AF1 Stepper H step
//      |
// TIM3_CH2 PC7  AF2 Stepper H count

namespace RotatingPlatform
{
static constexpr float MAX_MAX_SPEED =
    0.03;  // Structurally not go more than this
static constexpr float ACCELERATION = 0.01;  //[RPS^2]
static constexpr float TIME_WAIT_MS = 100;  //[MS]

/**
 * @brief Error handling enum for the stepper movement
 */
enum class ActuationStatus : uint8_t
{
    OK,              ///< `0`
    POSITION_LIMIT,  ///< `1` The STEPPER_X reached its actuation limits
    SPEED_LIMIT,     ///< `2` Stepper reached its velocity limits
    NOT_TEST,        ///< `3` Such movement is allowed only in test
    NO_STEPPER,      ///< `4` The specified stepper does not exist
    DISABLED,        ///< `5`
};

/**
 * @brief Actuators handles the stepper X and exposes functions to drive it.
 * Also, it updates its status by checking the speed and linearly increase and
 * decrease it. This is done to avoid issues due to inertia and for safety
 * reasons.
 */
class Actuators : public Boardcore::Injectable, public Boardcore::ActiveObject
{
public:
    /**
     * @brief Construct a new Actuators object using default stepper
     * configurations
     *
     */
    Actuators();

    /**
     * @brief Construct a new Actuators object using a given configuration for
     * the stepper
     *
     * @param config The stepper configuration, used to override the default one
     */
    Actuators(StepperConfig config);

    /**
     * @brief Enables the actuators
     */
    void arm();

    /**
     * @brief Disables the actuators
     */
    void disarm();

    /**
     * @brief Enables the rotational movement
     *
     */
    void enableRotation();

    /**
     * @brief Disables the rotational movement
     *
     */
    void disableRotation();

    ActuationStatus setSpeed(StepperList axis, float speed);
    ActuationStatus move(StepperList axis, int16_t steps);
    ActuationStatus moveDeg(StepperList axis, float degrees);
    ActuationStatus setPosition(StepperList axis, int16_t steps);
    ActuationStatus setPositionDeg(StepperList axis, float degrees);

    void setMultipliers(StepperList axis, float multiplier);
    bool areMultipliersSet();

    void zeroPosition();

    int16_t getCurrentPosition(StepperList axis);
    float getCurrentDegPosition(StepperList axis);

    /**
     * @brief Getter for the last actuation of the wanted stepper.
     * @param axis The stepper from which we want this information.
     * @returns The last delta angle the chosen stepper is performing [deg].
     */
    float getDeltaAngleDeg(StepperList axis) const
    {
        switch (axis)
        {
            case StepperList::STEPPER_X:
                return deltaX;
            default:
                assert(false && "Non existent stepper");
                return 0;
        }
    }

    /**
     * @brief Getter for the speed of the wanted stepper.
     * @param axis The stepper from which we want this information.
     * @returns The speed of the chosen stepper [rps].
     */
    float getSpeed(StepperList axis) const
    {
        switch (axis)
        {
            case StepperList::STEPPER_X:
                return speedX;
            default:
                assert(false && "Non existent stepper");
                return 0;
        }
    }

protected:
    /**
     * @brief Run function for actuators in rotational platform case. It
     * accelerate and then spin to the wanted rotational speed
     * @note This function is used to linearly increase and decrease the speed
     * of the steppers.
     */
    void run() override;

private:
    Boardcore::StepperPWM* getStepper(StepperList stepper)
    {
        switch (stepper)
        {
            case StepperList::STEPPER_X:
                return &stepperX;
            default:
                assert(false && "Non existent stepper");
                return nullptr;
        }
    };

    const StepperConfig* getStepperConfig(StepperList stepper) const
    {
        switch (stepper)
        {
            case StepperList::STEPPER_X:
                return &RotatingPlatform::Config::stepperXConfig;

            default:
                assert(false && "Non existent stepperConfig");
                return nullptr;
        }
    };

    float getStepperMultiplier(StepperList stepper) const
    {
        switch (stepper)
        {
            case StepperList::STEPPER_X:
                return multiplierX;
            default:
                assert(false && "Non existent stepperConfig");
                return 0;
        }
    };

    void logStepperData(StepperList stepper, Boardcore::StepperData data)
    {
        switch (stepper)
        {
            case StepperList::STEPPER_X:
                Boardcore::Logger::getInstance().log(
                    static_cast<StepperXData>(data));
                break;
            default:
                assert(false && "Non existent stepper");
                break;
        }
    }

    Boardcore::StepperPWM stepperX;

    // Flags indicating if the multipliers have been set
    float multiplierX = 2.2f;  // Multiplier for the stepper X
    float multiplierY = 2.2f;  // Multiplier for the stepper Y

    float deltaX = 0.0f;  // Delta angle to perform [deg]
    float deltaY = 0.0f;  // Delta angle to perform [deg]
    float speedX = 0;     // Speed of the stepper [rps]

    StepperConfig configX = Config::stepperXConfig;

    // Variables to enable the rotation
    bool isRotating = false;
    miosix::FastMutex rotationMutex;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Actuator");

    float speed = 0;
};
}  // namespace RotatingPlatform
