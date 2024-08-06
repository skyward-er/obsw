/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

#include <common/Events.h>
#include <common/Topics.h>
#include <drivers/canbus/CanDriver/CanDriver.h>

#include <cstdint>
#include <map>

namespace Common
{

namespace CanConfig
{

constexpr uint32_t BAUD_RATE = 250 * 1000;
constexpr float SAMPLE_POINT = 87.5f / 100.0f;

constexpr Boardcore::Canbus::CanbusDriver::AutoBitTiming BIT_TIMING = {
    .baudRate = BAUD_RATE, .samplePoint = SAMPLE_POINT};

constexpr Boardcore::Canbus::CanbusDriver::CanbusConfig CONFIG;

enum class Priority : uint8_t
{
    CRITICAL = 0,
    HIGH,
    MEDIUM,
    LOW
};

enum class PrimaryType : uint8_t
{
    EVENTS = 0,
    SENSORS,
    STATUS,
    COMMAND,
    ACTUATORS
};

// Used for source and destination
enum class Board : uint8_t
{
    BROADCAST = 0,
    RIG,
    MAIN,
    PAYLOAD,
    MOTOR
};

enum class SensorId : uint8_t
{
    PITOT,
    CC_PRESSURE,
    BOTTOM_TANK_PRESSURE,
    TOP_TANK_PRESSURE,
    TANK_TEMPERATURE,
    MOTOR_ACTUATORS_CURRENT,
    MAIN_BOARD_CURRENT,
    PAYLOAD_BOARD_CURRENT,
    MOTOR_BOARD_VOLTAGE
};

enum class EventId : uint8_t
{
    FORCE_INIT = 0,
    ARM,
    ENTER_TEST_MODE,
    EXIT_TEST_MODE,
    ENTER_HIL_MODE,
    EXIT_HIL_MODE,
    CALIBRATE,
    DISARM,
    LIFTOFF,
    APOGEE_DETECTED,
    IGNITION,
};

static const std::map<Common::CanConfig::EventId, Common::Events> eventToEvent{
    {Common::CanConfig::EventId::ARM, Common::CAN_ARM},
    {Common::CanConfig::EventId::DISARM, Common::CAN_DISARM},
    {Common::CanConfig::EventId::CALIBRATE, Common::CAN_CALIBRATE},
    {Common::CanConfig::EventId::LIFTOFF, Common::CAN_LIFTOFF},
    {Common::CanConfig::EventId::FORCE_INIT, Common::CAN_FORCE_INIT},
    {Common::CanConfig::EventId::ENTER_TEST_MODE, Common::CAN_ENTER_TEST_MODE},
    {Common::CanConfig::EventId::EXIT_TEST_MODE, Common::CAN_EXIT_TEST_MODE},
    {Common::CanConfig::EventId::ENTER_HIL_MODE, Common::CAN_ENTER_HIL_MODE},
    {Common::CanConfig::EventId::EXIT_HIL_MODE, Common::CAN_EXIT_HIL_MODE},
    {Common::CanConfig::EventId::APOGEE_DETECTED, Common::CAN_APOGEE_DETECTED},
    {Common::CanConfig::EventId::IGNITION, Common::CAN_IGNITION},
};

}  // namespace CanConfig

}  // namespace Common
