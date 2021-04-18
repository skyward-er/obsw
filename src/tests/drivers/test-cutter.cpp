/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

//#include <interfaces-impl/hwmapping.h>
#include <miosix.h>

#include <iostream>
#include <sstream>
//#include "configs/CutterConfig.h"
#include "drivers/adc/InternalADC/InternalADC.h"
#include "drivers/hbridge/HBridge.h"
#include "sensors/analog/current/CurrentSensor.h"

#include "Common.h"

using namespace std;
using namespace miosix;
// using namespace DeathStackBoard;

/**
 * PWM    : PE6 (both primary and backup)
 * ENA    : PD11 (primary)
 *          PG2  (backup)
 * Csense : PF6 (primary)
 *          PF8 (backup)
 */

bool PRINT                       = true;
static constexpr int CUT_TIME    = 10 * 1000;  // ms
static constexpr int CSENSE_FREQ = 50;         // hz

long long measured_cut_time = 0;
bool finished               = false;

InternalADC::Channel ADC_CHANNEL_PRIMARY = InternalADC::Channel::CH4;
InternalADC::Channel ADC_CHANNEL_BACKUP  = InternalADC::Channel::CH6;
ADC_TypeDef& ADCx                        = *ADC3;
InternalADC adc(ADCx);

static const PWM::Timer CUTTER_TIM{
    TIM9, &(RCC->APB2ENR), RCC_APB2ENR_TIM9EN,
    TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB2)};

static const PWMChannel CUTTER_CHANNEL_PRIMARY = PWMChannel::CH2;
static const PWMChannel CUTTER_CHANNEL_BACKUP  = PWMChannel::CH2;

using pwm = Gpio<GPIOE_BASE, 6>;

using ena_primary   = Gpio<GPIOD_BASE, 11>;
using csens_primary = Gpio<GPIOF_BASE, 6>;

using ena_backup   = Gpio<GPIOG_BASE, 2>;
using csens_backup = Gpio<GPIOF_BASE, 8>;

HBridge* cutter;

void wait(void* arg)
{
    UNUSED(arg);
    
    long long t  = getTick();
    long long t0 = t;

    while (t < t0 + CUT_TIME)
    {
        Thread::sleep(50);

        t = getTick();

        if (PRINT)
        {
            printf("Elapsed time : %.2f \n", (t - t0) / 1000.0);
        }
    }

    measured_cut_time = t - t0;

    Thread::sleep(100);

    if (!finished)
    {
        finished = true;
        cutter->disable();
        printf("\nTimer elapsed... Cutter disabled \n");
        printf("Press ENTER to exit \n");
    }
}

void csense(void* args)
{
    unsigned int c = *(unsigned int*)args;

    std::function<ADCData()> get_voltage_function;

    if (c == 1)
    {
        get_voltage_function = []() {
            return adc.getVoltage(ADC_CHANNEL_PRIMARY);
        };
    }
    else
    {
        get_voltage_function = []() {
            return adc.getVoltage(ADC_CHANNEL_BACKUP);
        };
    }

    std::function<float(float)> adc_to_current = [](float adc_in) {
        return (adc_in - 107.0f) * 32.4f;
    };

    CurrentSensor current_sensor(get_voltage_function, adc_to_current);

    if (!adc.init() || !adc.selfTest())
    {
        printf("ERROR : %d\n", adc.getLastError());
    }

    current_sensor.init();

    while (!finished)
    {
        adc.sample();
        miosix::Thread::sleep(1000 / CSENSE_FREQ);
        current_sensor.sample();

        CurrentSenseData current_data = current_sensor.getLastSample();
        printf("Time: %llu - ADC channel: %u - V:%f - A:%f \n",
               current_data.adc_timestamp, current_data.channel_id,
               current_data.voltage, current_data.current);
    }
}

void init()
{
    // Enable ADC3 clock
    {
        miosix::FastInterruptDisableLock dLock;

        RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;  // <- CHANGE THIS!

        pwm::mode(Mode::ALTERNATE);
        pwm::alternateFunction(3);

        ena_primary::mode(Mode::OUTPUT);
        ena_primary::low();
        csens_primary::mode(Mode::INPUT_ANALOG);

        ena_backup::mode(Mode::OUTPUT);
        ena_backup::low();
        csens_backup::mode(Mode::INPUT_ANALOG);
    }

    TimestampTimer::enableTimestampTimer();

    adc.enableChannel(ADC_CHANNEL_PRIMARY);
    adc.enableChannel(ADC_CHANNEL_BACKUP);
}

int main()
{
    init();

    for (;;)
    {
        finished          = false;
        measured_cut_time = 0;

        printf("\nWhat do you want to cut? (1 - primary / 2 - backup)\n");

        unsigned int c, freq = 0;
        float duty = 0;
        string temp;
        getline(cin, temp);
        stringstream(temp) >> c;
        if (c != 1 && c != 2)
        {
            printf("Choose 1 or 2\n");
            continue;
        }
        printf("Insert frequency (1-30000 Hz): \n");
        getline(cin, temp);
        stringstream(temp) >> freq;

        printf("Insert duty cycle (1-100): \n");
        getline(cin, temp);
        stringstream(temp) >> duty;

        printf("Cutting %d, freq: %d, duty: %f\n\n", c, freq, duty);

        if (!(freq >= 1 && freq <= 30000 && duty >= 0.0f && duty <= 100.0f))
        {
            printf("Wrong inputs!\n\n");
            continue;
        }

        do
        {
            printf("READY!\nWrite 'start' to begin:\n");
            getline(cin, temp);
        } while (temp != "start");

        printf("\n---------- Press ENTER to stop ----------\n");
        printf("The PWM will automatically be disabled after %d s\n\n",
               CUT_TIME / 1000);

        Thread::create(csense, 2048, MAIN_PRIORITY, &c);

        {
            GpioPin* ena_pin;
            PWMChannel channel;

            if (c == 1)
            {
                ena_pin = new GpioPin(ena_primary::getPin());
                channel = CUTTER_CHANNEL_PRIMARY;
            }
            else  // if (c == 2)
            {
                ena_pin = new GpioPin(ena_backup::getPin());
                channel = CUTTER_CHANNEL_BACKUP;
            }

            cutter = new HBridge(*ena_pin, CUTTER_TIM, channel, freq,
                                 duty / 100.0f, 50);

            cutter->enable();
            Thread::create(wait, 2048);

            // wait for the user to press ENTER or the timer to elapse
            while (!finished)
            {
                getline(cin, temp);
                finished = true;
                printf("Stopped... \n\n");
            }
            cutter->disable();

            delete cutter;
        }

        Thread::sleep(500);

        printf("Cut Time: %.2f s\n\n", measured_cut_time / 1000.0f);
        printf("Done!\n\n");
        printf("----------------------------------------------------\n");
    }

    return 0;
}