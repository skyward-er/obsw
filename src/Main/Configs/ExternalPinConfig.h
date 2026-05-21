/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus
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

#include <drivers/MCP23S17/MCP23S17.h>

namespace Main
{
namespace Config
{
namespace ExternalPin
{
// Port A
static constexpr Boardcore::ExternalGpioPin EXT_STATUS_LED(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN0);
static constexpr Boardcore::ExternalGpioPin DETACH_NOSECONE(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN1);
static constexpr Boardcore::ExternalGpioPin EXPULSION_SENSE(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN2);
static constexpr Boardcore::ExternalGpioPin RELEASER_SENSE(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN3);
static constexpr Boardcore::ExternalGpioPin LED_0(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN4);
static constexpr Boardcore::ExternalGpioPin LED_1(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN5);
static constexpr Boardcore::ExternalGpioPin LED_2(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN6);
static constexpr Boardcore::ExternalGpioPin VN100_SYNC_OUT(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN7);

// Port B
static constexpr Boardcore::ExternalGpioPin VN100_SYNC_IN(
    Boardcore::MCP23S17Defs::PORT::PORT_B, Boardcore::MCP23S17Defs::PIN::PIN0);
static constexpr Boardcore::ExternalGpioPin CUBESAT_ENABLE(
    Boardcore::MCP23S17Defs::PORT::PORT_B, Boardcore::MCP23S17Defs::PIN::PIN1);
static constexpr Boardcore::ExternalGpioPin CAM_ENABLE(
    Boardcore::MCP23S17Defs::PORT::PORT_B, Boardcore::MCP23S17Defs::PIN::PIN2);
static constexpr Boardcore::ExternalGpioPin RF_NRST(
    Boardcore::MCP23S17Defs::PORT::PORT_B, Boardcore::MCP23S17Defs::PIN::PIN3);
static constexpr Boardcore::ExternalGpioPin RF_FLASH_CS(
    Boardcore::MCP23S17Defs::PORT::PORT_B, Boardcore::MCP23S17Defs::PIN::PIN4);
static constexpr Boardcore::ExternalGpioPin RF_TX_ENABLE(
    Boardcore::MCP23S17Defs::PORT::PORT_B, Boardcore::MCP23S17Defs::PIN::PIN5);
static constexpr Boardcore::ExternalGpioPin RF_RX_ENABLE(
    Boardcore::MCP23S17Defs::PORT::PORT_B, Boardcore::MCP23S17Defs::PIN::PIN6);
static constexpr Boardcore::ExternalGpioPin RF_FE_PROT(
    Boardcore::MCP23S17Defs::PORT::PORT_B, Boardcore::MCP23S17Defs::PIN::PIN7);

}  // namespace ExternalPin
}  // namespace Config
}  // namespace Main
