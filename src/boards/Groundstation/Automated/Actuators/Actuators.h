/* Copyright (c) 2023-2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano, Nicolò Caruso
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

#include <common/Mavlink.h>
#include <logger/Logger.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "ActuatorsConfig.h"
#include "ActuatorsData.h"
#include "actuators/stepper/StepperPWM.h"

// TIM1_CH4 PA11 AF1
//      |
// TIM3_CH2 PC7  AF2

// TIM4_CH1 PD12 AF2
//      |
// TIM8_CH4 PC9  AF3

namespace Antennas
{

/**
 * @brief Error handling enum for the stepper movement
 */
enum class ErrorMovement : uint8_t
{
    OK,              ///< `0`
    LIMIT,           ///< `1` The STEPPER_X reached its actuation limits
    NOT_TEST,        ///< `2` Such movement is allowed only in test
    NO_STEPPER,      ///< `3` The specified stepper does not exist
    DISABLED,        ///< `4`
    EMERGENCY_STOP,  ///< `5`
};

class Actuators : public Boardcore::Module
{
public:
    Actuators();

    /**
     * @brief Dummy start for actuators
     * @note The real enable is done by the `arm()` method
     */
    void start();

    /**
     * @brief Enables the actuators
     */
    void arm();

    /**
     * @brief Disables the actuators
     */
    void disarm();

    void IRQemergencyStop();
    void IRQemergencyStopRecovery();

    void setSpeed(StepperList axis, float speed);

    ErrorMovement move(StepperList axis, int16_t steps);
    ErrorMovement moveDeg(StepperList axis, float degrees);
    ErrorMovement setPosition(StepperList axis, int16_t steps);
    ErrorMovement setPositionDeg(StepperList axis, float degrees);

    void zeroPosition();

    int16_t getCurrentPosition(StepperList axis);
    float getCurrentDegPosition(StepperList axis);

    bool isEmergencyStopped() const { return emergencyStop; }

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
            case StepperList::STEPPER_Y:
                return deltaY;
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
            case StepperList::STEPPER_Y:
                return speedY;
            default:
                assert(false && "Non existent stepper");
                return 0;
        }
    }

private:
    Boardcore::StepperPWM* getStepper(StepperList stepper)
    {
        switch (stepper)
        {
            case StepperList::STEPPER_X:
                return &stepperX;
            case StepperList::STEPPER_Y:
                return &stepperY;
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
                return &Antennas::Config::stepperXConfig;
            case StepperList::STEPPER_Y:
                return &Antennas::Config::stepperYConfig;
            default:
                assert(false && "Non existent stepperConfig");
                return nullptr;
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
            case StepperList::STEPPER_Y:
                Boardcore::Logger::getInstance().log(
                    static_cast<StepperYData>(data));
                break;
            default:
                assert(false && "Non existent stepper");
                break;
        }
    }

    Boardcore::StepperPWM stepperX;
    Boardcore::StepperPWM stepperY;

    float deltaX = 0.0f;  // Delta angle to perform [deg]
    float deltaY = 0.0f;  // Delta angle to perform [deg]
    float speedX =
        Config::stepperXConfig.MAX_SPEED;  // Speed of the stepper [rps]
    float speedY =
        Config::stepperYConfig.MAX_SPEED;  // Speed of the stepper [rps]

    bool emergencyStop =
        false;  // Whether the system performed an emergency stop
};
}  // namespace Antennas