/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Author: Alvise de' Faveri Tron
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

#include <stdint.h>

namespace CanInterfaces
{

/**
 * @brief CanTopics = Canbus FilterIds = Source of the Canbus message
 *
 * Pay attention to the ORDER: lower number => higher priority
 */
enum CanTopic : uint16_t
{
    CAN_TOPIC_HOMEONE  = 0,
    CAN_TOPIC_LAUNCH   = 2,
    CAN_TOPIC_IGNITION = 4,
    CAN_TOPIC_NOSECONE = 8
};

enum CanCommandID : uint8_t
{
    CAN_MSG_ABORT,
    CAN_MSG_REQ_IGN_STATUS,
    CAN_MSG_REQ_NSC_STATUS,
    CAN_MSG_NSC_OPEN,
    CAN_MSG_NSC_CLOSE,
    CAN_MSG_NSC_STOP
};

/**
 * @brief Boards Status structs.
 */
struct __attribute__((packed)) IgnitionBoardStatus
{
    uint8_t avr_abortCmd : 1;        // abort after explicit command (LSB)
    uint8_t avr_abortTimeout : 1;    // abort after timeout
    uint8_t avr_abortWrongCode : 1;  // abort after wrong launch code received
    uint8_t avr_launchDone : 1;
    uint8_t avr_powerOnReset : 1;   // last AVR reset cause was power on
    uint8_t avr_externalReset : 1;  // last AVR reset cause was reset pin
    uint8_t avr_brownoutReset : 1;  // last AVR reset cause was brownout
    uint8_t avr_watchdogReset : 1;  // last AVR reset cause was watchdog (MSB)

    uint8_t stm32_abortCmd : 1;        // abort after explicit command (LSB)
    uint8_t stm32_abortTimeout : 1;    // abort after timeout
    uint8_t stm32_abortWrongCode : 1;  // abort after wrong launch code received
    uint8_t stm32_launchDone : 1;
    uint8_t stm32_powerOnReset : 1;   // last STM32 reset cause was power on
    uint8_t stm32_externalReset : 1;  // last STM32 reset cause was reset pin
    uint8_t stm32_brownoutReset : 1;  // last STM32 reset cause was brownout
    uint8_t
        stm32_watchdogReset : 1;  // last STM32 reset cause was watchdog (MSB)
};

} /* namespace CanInterfaces */
