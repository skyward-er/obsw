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

#include <Main/Buses.h>
#include <Main/Configs/GpioExpanderConfig.h>
#include <drivers/MCP23S17/MCP23S17.h>
#include <interfaces-impl/hwmapping.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Main
{

class GpioExpander : public Boardcore::InjectableWithDeps<Buses>
{
public:
    bool start()
    {
        expander = new Boardcore::MCP23S17(getModule<Buses>()->getExpander(),
                                           miosix::expander::cs::getPin());

        expander->init();
        expander->setPinMode(Config::GpioExpander::EXT_STATUS_LED.getPort(),
                             Config::GpioExpander::EXT_STATUS_LED.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);
        expander->setPinMode(Config::GpioExpander::DETACH_PAYLOAD.getPort(),
                             Config::GpioExpander::DETACH_PAYLOAD.getPin(),
                             Boardcore::MCP23S17Defs::MODE::INPUT);
        expander->setPinMode(Config::GpioExpander::EXPULSION_SENSE.getPort(),
                             Config::GpioExpander::EXPULSION_SENSE.getPin(),
                             Boardcore::MCP23S17Defs::MODE::INPUT);
        expander->setPinMode(Config::GpioExpander::RELEASER_SENSE.getPort(),
                             Config::GpioExpander::RELEASER_SENSE.getPin(),
                             Boardcore::MCP23S17Defs::MODE::INPUT);
        expander->setPinMode(Config::GpioExpander::LED_0.getPort(),
                             Config::GpioExpander::LED_0.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);
        expander->setPinMode(Config::GpioExpander::LED_1.getPort(),
                             Config::GpioExpander::LED_1.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);
        expander->setPinMode(Config::GpioExpander::LED_2.getPort(),
                             Config::GpioExpander::LED_2.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);

        expander->setPinMode(Config::GpioExpander::CUBESAT_ENABLE.getPort(),
                             Config::GpioExpander::CUBESAT_ENABLE.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);
        expander->setPinMode(Config::GpioExpander::CAM_ENABLE.getPort(),
                             Config::GpioExpander::CAM_ENABLE.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);
        expander->setPinMode(Config::GpioExpander::RF_NRST.getPort(),
                             Config::GpioExpander::RF_NRST.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);
        expander->setPinMode(Config::GpioExpander::RF_FLASH_CS.getPort(),
                             Config::GpioExpander::RF_FLASH_CS.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);
        expander->setPinValue(Config::GpioExpander::RF_FLASH_CS.getPort(),
                              Config::GpioExpander::RF_FLASH_CS.getPin(), true);
        expander->setPinMode(Config::GpioExpander::RF_TX_ENABLE.getPort(),
                             Config::GpioExpander::RF_TX_ENABLE.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);
        expander->setPinMode(Config::GpioExpander::RF_RX_ENABLE.getPort(),
                             Config::GpioExpander::RF_RX_ENABLE.getPin(),
                             Boardcore::MCP23S17Defs::MODE::OUTPUT);
        expander->setPinMode(Config::GpioExpander::RF_FE_PROT.getPort(),
                             Config::GpioExpander::RF_FE_PROT.getPin(),
                             Boardcore::MCP23S17Defs::MODE::INPUT);

        return true;
    }

    Boardcore::MCP23S17& getExpander() { return *expander; }

private:
    Boardcore::MCP23S17* expander;
};

}  // namespace Main
