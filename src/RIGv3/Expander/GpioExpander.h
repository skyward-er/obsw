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

#include <RIGv3/Buses.h>
#include <RIGv3/Configs/GpioExpanderConfig.h>
#include <drivers/MCP23S17/MCP23S17.h>
#include <utils/DependencyManager/DependencyManager.h>

// using namespace Config::GpioExpander;

namespace RIGv3
{

class GpioExpander : public Boardcore::InjectableWithDeps<Buses>
{
public:
    bool start()
    {
        expander =
            new Boardcore::MCP23S17(getModule<Buses>()->getExpander(),
                                    getModule<Buses>()->getGpioExpanderCsPin());

        expander->init();
        expander->setPinMode(Config::GpioExpander::DWS_ENABLE_PIN.getPort(),
                             Config::GpioExpander::DWS_ENABLE_PIN.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);
        // DWS enable is normally high since it powers DWS system
        expander->setPinValue(Config::GpioExpander::DWS_ENABLE_PIN.getPort(),
                              Config::GpioExpander::DWS_ENABLE_PIN.getPin(), 1);

        expander->setPinMode(Config::GpioExpander::PRZ_DET_VALVE_PIN.getPort(),
                             Config::GpioExpander::PRZ_DET_VALVE_PIN.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);
        expander->setPinMode(Config::GpioExpander::OX_DET_VALVE_PIN.getPort(),
                             Config::GpioExpander::OX_DET_VALVE_PIN.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);
        expander->setPinMode(Config::GpioExpander::PURGE_VALVE_PIN.getPort(),
                             Config::GpioExpander::PURGE_VALVE_PIN.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);
        expander->setPinMode(Config::GpioExpander::ARMING_LIGHT_PIN.getPort(),
                             Config::GpioExpander::ARMING_LIGHT_PIN.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);

        return true;
    }

    Boardcore::MCP23S17& getExpander() { return *expander; }

private:
    Boardcore::MCP23S17* expander;
};

}  // namespace RIGv3
