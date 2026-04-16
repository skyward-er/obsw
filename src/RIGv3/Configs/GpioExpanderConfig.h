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

namespace RIGv3
{
namespace Config
{
namespace GpioExpander
{

static constexpr Boardcore::ExternalGpioPin DWS_ENABLE_PIN(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN0);
static constexpr Boardcore::ExternalGpioPin PRZ_DET_VALVE_PIN(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN1);
static constexpr Boardcore::ExternalGpioPin PURGE_VALVE_PIN(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN2);
static constexpr Boardcore::ExternalGpioPin OX_DET_VALVE_PIN(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN3);
static constexpr Boardcore::ExternalGpioPin ARMING_LIGHT_PIN(
    Boardcore::MCP23S17Defs::PORT::PORT_A, Boardcore::MCP23S17Defs::PIN::PIN5);

}  // namespace GpioExpander
}  // namespace Config
}  // namespace RIGv3
