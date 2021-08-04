/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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

/**
 * Components on the power board:
 *
 * Analog input: Battery meter
 * HBridge (x2): Cutters (primary & backup)
 * Servo (x2): Deployment & aerobrakes
 */

#include <AirBrakes/AirBrakesServo.h>
#include <Common.h>
#include <Deployment/DeploymentServo.h>
#include <drivers/adc/InternalADC/InternalADC.h>
#include <drivers/hbridge/HBridge.h>
#include <drivers/servo/servo.h>
#include <sensors/analog/battery/BatteryVoltageSensor.h>

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

namespace CutterTest
{
#include "../drivers/test-cutter.cpp"
}

namespace ServoTest
{
#include "../drivers/test-servo.cpp"
}

using namespace std;

// Sample frequency
constexpr int SAMPLING_FREQUENCY = 20;

int menu();
int askSeconds();
void sampleBatteryVoltage();

int main()
{
    TimestampTimer::enableTimestampTimer();

    switch (menu())
    {
        case 1:
            sampleBatteryVoltage();
            break;
        case 2:
            CutterTest::main();
            break;
        case 3:
            ServoTest::main();
            break;

        default:
            break;
    }

    return 0;
}

int menu()
{
    string temp;
    int choice;

    printf("\n\nWhat do you want to do?\n");
    printf("1. Sample battery voltage\n");
    printf("2. Test cutters\n");
    printf("3. Test servos\n");
    printf("\n>> ");
    getline(cin, temp);
    stringstream(temp) >> choice;

    return choice;
}

int askSeconds()
{
    int seconds;

    printf("How many seconds the test should run?\n");
    printf("\n>> ");
    scanf("%d", &seconds);

    return seconds;
}

void sampleBatteryVoltage()
{
    // Set the clock divider for the analog circuitry (/8)
    ADC->CCR |= ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1;

    // Ask the user how many second the test should be perfomed
    int seconds = askSeconds();

    // Sensor setup

    InternalADC internalADC = InternalADC(*ADC3, 3.3);
    internalADC.enableChannel(InternalADC::CH5);
    internalADC.init();

    std::function<ADCData()> get_voltage_function =
        std::bind(&InternalADC::getVoltage, &internalADC, InternalADC::CH5);

    BatteryVoltageSensor batterySensor(get_voltage_function, 5.98);

    // Sampling
    printf("adc_timestamp,gpio_voltage,bat_voltage,battery_percentage\n");
    for (int i = 0; i < seconds * SAMPLING_FREQUENCY; i++)
    {
        internalADC.sample();
        batterySensor.sample();
        BatteryVoltageSensorData data = batterySensor.getLastSample();

        // Calculate a simple linear battery percentage
        float batteryPercentage = 100 * (data.bat_voltage - 9.6) / (12.6 - 9.6);

        printf("%llu,%.3f,%.3f,%.1f\n", data.adc_timestamp, data.voltage,
               data.bat_voltage, batteryPercentage);

        miosix::delayMs(1000 / SAMPLING_FREQUENCY);
    }
}
