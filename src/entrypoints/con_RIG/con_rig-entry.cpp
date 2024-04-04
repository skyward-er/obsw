/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Giacomo Caironi
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

#include <con_RIG/BoardScheduler.h>
#include <con_RIG/Buses.h>
#include <con_RIG/Buttons/Buttons.h>
#include <con_RIG/Configs/ButtonsConfig.h>
#include <con_RIG/Radio/Radio.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>

#include <thread>
#include <utils/ModuleManager/ModuleManager.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace con_RIG;

// TODO delete with hwmapping and bsp
void initPins()
{
    // SPI1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    using sck  = Gpio<GPIOA_BASE, 5>;
    using miso = Gpio<GPIOA_BASE, 6>;
    using mosi = Gpio<GPIOA_BASE, 7>;
    using cs   = Gpio<GPIOF_BASE, 6>;

    using dio0 = Gpio<GPIOB_BASE, 1>;
    using dio1 = Gpio<GPIOD_BASE, 12>;
    using dio3 = Gpio<GPIOD_BASE, 13>;

    using txEn = Gpio<GPIOG_BASE, 2>;
    using rxEn = Gpio<GPIOG_BASE, 3>;
    using nrst = Gpio<GPIOB_BASE, 0>;

    sck::mode(Mode::ALTERNATE);
    miso::mode(Mode::ALTERNATE);
    mosi::mode(Mode::ALTERNATE);

    sck::alternateFunction(5);
    miso::alternateFunction(5);
    mosi::alternateFunction(5);

    cs::mode(Mode::OUTPUT);
    dio0::mode(Mode::INPUT_PULL_UP);
    dio1::mode(Mode::INPUT_PULL_UP);
    dio3::mode(Mode::INPUT_PULL_UP);
    txEn::mode(Mode::OUTPUT);
    rxEn::mode(Mode::OUTPUT);
    nrst::mode(Mode::OUTPUT);

    cs::high();
    nrst::high();

    // USART 1
    using tx = Gpio<GPIOA_BASE, 9>;
    using rx = Gpio<GPIOA_BASE, 10>;

    tx::mode(Mode::ALTERNATE);
    rx::mode(Mode::ALTERNATE);

    tx::alternateFunction(7);
    rx::alternateFunction(7);

    // Buttons and switch
    using GpioIgnitionBtn        = Gpio<GPIOB_BASE, 4>;
    using GpioFillingValveBtn    = Gpio<GPIOE_BASE, 6>;
    using GpioVentingValveBtn    = Gpio<GPIOE_BASE, 4>;
    using GpioReleasePressureBtn = Gpio<GPIOG_BASE, 9>;
    using GpioQuickConnectorBtn  = Gpio<GPIOD_BASE, 7>;
    using GpioStartTarsBtn       = Gpio<GPIOD_BASE, 5>;
    using GpioArmedSwitch        = Gpio<GPIOE_BASE, 2>;

    GpioIgnitionBtn::mode(Mode::INPUT);
    GpioFillingValveBtn::mode(Mode::INPUT);
    GpioVentingValveBtn::mode(Mode::INPUT);
    GpioReleasePressureBtn::mode(Mode::INPUT);
    GpioQuickConnectorBtn::mode(Mode::INPUT);
    GpioStartTarsBtn::mode(Mode::INPUT);
    GpioArmedSwitch::mode(Mode::INPUT);

    // Actuators
    using armed_led = Gpio<GPIOC_BASE, 13>;
    using buzzer    = Gpio<GPIOB_BASE, 7>;
    using redLed    = Gpio<GPIOG_BASE, 14>;  // Red LED

    armed_led::mode(Mode::OUTPUT);
    buzzer::mode(Mode::OUTPUT);
    redLed::mode(Mode::OUTPUT);

    buzzer::high();
}

int main()
{
    PrintLogger logger     = Logging::getLogger("main");
    ModuleManager& modules = ModuleManager::getInstance();

    BoardScheduler* scheduler = new BoardScheduler();
    Buses* buses              = new Buses();
    Radio* radio              = new Radio(scheduler->getRadioScheduler());
    Buttons* buttons          = new Buttons(scheduler->getButtonsScheduler());

    bool initResult = modules.insert<BoardScheduler>(scheduler) &&
                      modules.insert<Buses>(buses) &&
                      modules.insert<Radio>(radio) &&
                      modules.insert<Buttons>(buttons);

    if (!radio->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the radio");
    }

    if (!buttons->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the buttons");
    }

    if (!scheduler->start())
    {
        initResult = false;
        LOG_ERR(logger, "Error starting the General Purpose Scheduler");
    }

    if (!initResult)
    {
        ui::redLed::high();
        LOG_ERR(logger, "Init failure!");
    }
    else
    {
        LOG_INFO(logger, "All good!");
    }

    // Periodical statistics
    while (true)
    {
        Thread::sleep(1000);
        CpuMeter::resetCpuStats();
    }
}
