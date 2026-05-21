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
#include <Main/Configs/ExternalPinConfig.h>
#include <drivers/MCP23S17/MCP23S17.h>
#include <interfaces-impl/hwmapping.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Main
{

class GpioExpander : public Boardcore::InjectableWithDeps<Buses>
{
#define SET_PIN_MODE(expander, name, mode)                    \
    expander->setPinMode(Config::ExternalPin::name.getPort(), \
                         Config::ExternalPin::name.getPin(),  \
                         Boardcore::MCP23S17Defs::MODE::mode);

#define SET_PIN_VALUE(expander, name, value)                   \
    expander->setPinValue(Config::ExternalPin::name.getPort(), \
                          Config::ExternalPin::name.getPin(), value);

public:
    bool start()
    {
        expander = new Boardcore::MCP23S17(getModule<Buses>()->getExpander(),
                                           miosix::expander::cs::getPin());

        expander->init();

        SET_PIN_MODE(expander, EXT_STATUS_LED, OUTPUT);
        SET_PIN_MODE(expander, DETACH_NOSECONE, INPUT);
        SET_PIN_MODE(expander, EXPULSION_SENSE, INPUT);
        SET_PIN_MODE(expander, RELEASER_SENSE, INPUT);
        SET_PIN_MODE(expander, LED_0, OUTPUT);
        SET_PIN_MODE(expander, LED_1, OUTPUT);
        SET_PIN_MODE(expander, LED_2, OUTPUT);
        SET_PIN_MODE(expander, VN100_SYNC_OUT, OUTPUT);

        SET_PIN_MODE(expander, VN100_SYNC_IN, INPUT);
        SET_PIN_MODE(expander, CUBESAT_ENABLE, OUTPUT);
        SET_PIN_MODE(expander, CAM_ENABLE, OUTPUT);
        SET_PIN_MODE(expander, RF_NRST, OUTPUT);
        SET_PIN_MODE(expander, RF_FLASH_CS, OUTPUT);
        SET_PIN_MODE(expander, RF_TX_ENABLE, OUTPUT);
        SET_PIN_MODE(expander, RF_RX_ENABLE, OUTPUT);
        SET_PIN_MODE(expander, RF_FE_PROT, OUTPUT);

        SET_PIN_VALUE(expander, EXT_STATUS_LED, false);
        SET_PIN_VALUE(expander, LED_0, false);
        SET_PIN_VALUE(expander, LED_1, false);
        SET_PIN_VALUE(expander, LED_2, false);
        SET_PIN_VALUE(expander, VN100_SYNC_OUT, false);
        SET_PIN_VALUE(expander, RF_FLASH_CS, true);

        return true;
    }

    Boardcore::MCP23S17& getExpander() { return *expander; }

    void camOn() { SET_PIN_VALUE(expander, CAM_ENABLE, true); }
    void camOff() { SET_PIN_VALUE(expander, CAM_ENABLE, false); }
    bool getCamState()
    {
        return expander->getPinValue(Config::ExternalPin::CAM_ENABLE.getPort(),
                                     Config::ExternalPin::CAM_ENABLE.getPin());
    }

    void statusLedOn() { SET_PIN_VALUE(expander, EXT_STATUS_LED, true); }
    void statusLedOff() { SET_PIN_VALUE(expander, EXT_STATUS_LED, false); }

private:
    Boardcore::MCP23S17* expander;
};

}  // namespace Main
