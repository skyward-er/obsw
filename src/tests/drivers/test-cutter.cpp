/* Copyright (c) 2018-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio
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

#include <Common.h>
#include <configs/CutterConfig.h>
#include <drivers/adc/InternalADC/InternalADC.h>
#include <drivers/hbridge/HBridge.h>
#include <sensors/analog/current/CurrentSensor.h>

#include <iostream>
#include <sstream>

/**
 * Pin map for DeathStackX
 *
 * PWM    : PE6 (both primary and backup)
 * ENA    : PD11 (primary)
 *          PG2  (backup)
 * Csense : PF6  (primary)
 *          PF8  (backup)
 */

using namespace std;

static constexpr int MAX_CUTTING_TIME = 10 * 1000;  // ms
constexpr int SAMPLING_FREQUENCY      = 20;

// This could go into CutterConfig
static constexpr InternalADC::Channel ADC_CHANNEL_PRIMARY =
    InternalADC::Channel::CH6;
static constexpr InternalADC::Channel ADC_CHANNEL_BACKUP =
    InternalADC::Channel::CH4;

// Check page 16 of datasheet for more informations
static constexpr float ADC_TO_CURR_DKILIS = 10204.85;  // Typ: 19.5
static constexpr float ADC_TO_CURR_RIS    = 510;
static constexpr float ADC_TO_CURR_IISOFF = .0001662;  // Typ: 170uA

static constexpr float ADC_TO_CURR_COEFF = ADC_TO_CURR_DKILIS / ADC_TO_CURR_RIS;
static constexpr float ADC_TO_CURR_OFFSET =
    ADC_TO_CURR_DKILIS * ADC_TO_CURR_IISOFF;

function<float(float)> adc_to_current = [](float adc_in)
{
    return ADC_TO_CURR_DKILIS * (adc_in / ADC_TO_CURR_RIS - ADC_TO_CURR_IISOFF);
};

bool finished = false;

void menu(unsigned int *cutterNo, uint32_t *frequency, float *dutyCycle);

void elapsedTimeAndCsense(void *args);

int main()
{
    unsigned int cutterNo = 0;
    uint32_t frequency    = 0;
    float dutyCycle       = 0;

    TimestampTimer::enableTimestampTimer();

    // Set the clock divider for the analog circuitry (/8)
    ADC->CCR |= ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1;

    // Ask the user the parameters
    menu(&cutterNo, &frequency, &dutyCycle);

    // Cutter setup

    GpioPin *ena_pin;
    PWMChannel pwmChannel;

    if (cutterNo == 1)
    {
        ena_pin = new GpioPin(
            DeathStackBoard::CutterConfig::PrimaryCutterEna::getPin());
        pwmChannel = DeathStackBoard::CutterConfig::CUTTER_CHANNEL_PRIMARY;
    }
    else  // if (cutterNo == 2)
    {
        ena_pin = new GpioPin(
            DeathStackBoard::CutterConfig::BackupCutterEna::getPin());
        pwmChannel = DeathStackBoard::CutterConfig::CUTTER_CHANNEL_BACKUP;
    }

    HBridge cutter(*ena_pin, DeathStackBoard::CutterConfig::CUTTER_TIM,
                   pwmChannel, frequency, dutyCycle / 100.0f);

    // Start the test

    miosix::Thread::create(elapsedTimeAndCsense, 2048, miosix::MAIN_PRIORITY,
                           &cutterNo);

    cutter.enable();

    // Wait for the user to press ENTER or the timer to elapse
    string temp;
    while (!finished)
    {
        (void)getchar();
        finished = true;
        printf("Stopped... \n\n");
    }

    cutter.disable();

    return 0;
}

void menu(unsigned int *cutterNo, uint32_t *frequency, float *dutyCycle)
{
    string temp;

    do
    {
        printf("\nWhat do you want to cut? (1 - primary / 2 - backup)\n");
        printf(">> ");
        getline(cin, temp);
        stringstream(temp) >> *cutterNo;
    } while (*cutterNo != 1 && *cutterNo != 2);

    do
    {
        printf("\nInsert frequency (1-30000 Hz): \n");
        printf(">> ");
        getline(cin, temp);
        stringstream(temp) >> *frequency;
    } while (*frequency < 1 || *frequency > 30000);

    do
    {
        printf("\nInsert duty cycle (1-100): \n");
        printf(">> ");
        getline(cin, temp);
        stringstream(temp) >> *dutyCycle;
    } while (*dutyCycle < 1 || *dutyCycle > 100);

    printf("\nCutting %s, frequency: %lu, duty cycle: %.1f\n\n",
           (*cutterNo ? "PRIMARY" : "BACKUP"), *frequency, *dutyCycle);

    do
    {
        printf("READY!\n");
        printf("Write 'start' to begin then press ENTER to end the test:\n");
        getline(cin, temp);
    } while (temp != "start");
}

void elapsedTimeAndCsense(void *args)
{
    int cutterNo = *(unsigned int *)args;
    // Sensors setup

    InternalADC internalADC(*ADC3, 3.3);
    internalADC.init();
    internalADC.enableChannel(ADC_CHANNEL_PRIMARY);
    internalADC.enableChannel(ADC_CHANNEL_BACKUP);

    function<ADCData()> get_voltage_function;

    if (cutterNo == 1)
    {
        get_voltage_function =
            bind(&InternalADC::getVoltage, &internalADC, ADC_CHANNEL_PRIMARY);
    }
    else
    {
        get_voltage_function =
            bind(&InternalADC::getVoltage, &internalADC, ADC_CHANNEL_BACKUP);
    }

    CurrentSensor current_sensor(get_voltage_function, adc_to_current);
    current_sensor.init();

    // Save the cuttent timestamp
    uint64_t t  = TimestampTimer::getTimestamp() / 1000;
    uint64_t t0 = t;

    while (t < t0 + MAX_CUTTING_TIME && !finished)
    {
        Thread::sleep(1000 / SAMPLING_FREQUENCY);

        t = TimestampTimer::getTimestamp() / 1000;
        internalADC.sample();
        current_sensor.sample();

        CurrentSenseData current_data = current_sensor.getLastSample();
        printf("Elapsed time : %.2f\tCsense: %.4fV %.3fA\n", (t - t0) / 1000.0,
               current_data.voltage, current_data.current);
    }

    printf("Cut Time: %.2f s\n\n", (t - t0) / 1000.0f);
}
