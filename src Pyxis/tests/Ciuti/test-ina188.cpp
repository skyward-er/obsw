/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <drivers/AD5204/AD5204.h>
#include <drivers/adc/InternalADC.h>
#include <miosix.h>
#include <utils/ButtonHandler/ButtonHandler.h>
#include <utils/Stats/Stats.h>

using namespace miosix;
using namespace Boardcore;

void buttonCallback(ButtonEvent event);
void print();

Stats ch2Stats;

int main()
{
    ADC->CCR |= ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1;

    InternalADC adc(ADC3, 3.289);
    adc.enableChannel(InternalADC::CH0);
    adc.enableChannel(InternalADC::CH1);
    adc.init();

    SPIBus bus(SPI2);
    SPIBusConfig config;
    AD5204 ad5204(bus, devices::ad5204::cs::getPin(), config,
                  AD5204::Resistance::R_10);

    ad5204.setResistance(AD5204::Channel::RDAC_3, 2040);
    buttonCallback(ButtonEvent::SHORT_PRESS);

    ButtonHandler::getInstance().registerButtonCallback(
        devices::buttons::record::getPin(), buttonCallback);

    TaskScheduler scheduler;
    scheduler.addTask(print, 250);
    scheduler.start();

    while (true)
    {
        adc.sample();
        ch2Stats.add(adc.getVoltage(InternalADC::CH1).voltage);

        Thread::sleep(1);
    }
}

void buttonCallback(ButtonEvent event)
{
    if (event == ButtonEvent::SHORT_PRESS)
    {
        static bool mosfetOn = !mosfetOn;

        if (mosfetOn)
        {
            devices::ina188::mosfet2::low();
            devices::leds::led1::high();
            printf("Mosfet turned on!\n");
        }
        else
        {
            devices::ina188::mosfet2::high();
            devices::leds::led1::low();
            printf("Mosfet turned off!\n");
        }
    }
}

void print()
{
    float ch2 = ch2Stats.getStats().mean;
    ch2Stats.reset();

    printf("INA: %6.3fmV -> Ponte: %6.3fmV\n", ch2 * 1000, ch2 * 1000 / 99.6);
}