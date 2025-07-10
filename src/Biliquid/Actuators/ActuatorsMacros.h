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

/**
 * @file ActuatorsMacros.h
 * @brief Macros for the Actuators module
 *
 * This file contains macros to simplify the creation of ValveInfo structs
 * for the Actuators module.
 *
 * The hardware mapping (hwmapping.h), the actuators configuration
 * (ActuatorsConfig.h) and the registry keys (Registry.h) have been created with
 * a specific naming convention to allow the use of these macros, to reduce the
 * amount of code repetition and to make the code more readable during servo
 * creation.
 *
 * The following naming convention is used:
 * - The servo name is an uppercase string code (e.g. OX_FIL)
 * - Constants are expected to be in the Config::Servos namespace
 * - The servo name is used to create the following constants:
 *      - MIOSIX_SERVOS_{servo}_TIM             (hwmapping)
 *      - MIOSIX_SERVOS_{servo}_CHANNEL         (hwmapping)
 *      - {servo}_LIMIT                         (ActuatorsConfig)
 *      - {servo}_FLIPPED                       (ActuatorsConfig)
 *      - Common::MOTOR_{servo}_OPEN            (Events)
 *      - Common::MOTOR_{servo}_CLOSE           (Events)
 *      - hwmapping::{servo}_LED                (hwmapping)
 *  - The following constants are used for all servos:
 *     - MIN_PULSE
 *     - MAX_PULSE
 *     - FREQUENCY
 */

/**
 * @brief Shorthand to create a non-atomic ValveInfo struct from the servo name
 */
#define MAKE_SIMPLE_SERVO(name)                                       \
    ValveInfo                                                         \
    {                                                                 \
        std::make_unique<Servo>(                                      \
            MIOSIX_SERVOS_##name##_TIM,                               \
            TimerUtils::Channel::MIOSIX_SERVOS_##name##_CHANNEL,      \
            Config::Servos::MIN_PULSE, Config::Servos::MAX_PULSE,     \
            Config::Servos::FREQUENCY),                               \
            ValveInfo::ValveConfig                                    \
        {                                                             \
            .id = Valve::name, .limit = Config::Servos::name##_LIMIT, \
            .flipped     = Config::Servos::name##_FLIPPED,            \
            .maxAperture = Config::Servos::name##_MAX_APERTURE,       \
        }                                                             \
    }
