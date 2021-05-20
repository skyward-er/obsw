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
 * This code allows testing of the whole analog board of the DeathStackX.
 *
 * When sampling all sensors the analog pressure sensors are sampled at 1/4 of
 * the configured frequency (due to the ads1118).
 *
 * Components on the analog board:
 *
 * Analog sensors:
 *     MPXHZ6130A (x2): Absolute pressure sensor
 *     SSCDANN030PAA: Absolute pressure sensor with a 0-206kPa range
 *     SSCDRRN015PDA: Differential pressure sensor with a ±103kPa range
 * MS5803: Digital absolute pressure sensor
 * Deatachment pins
 */

#include <Common.h>
#include <drivers/adc/ADS1118/ADS1118.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <sensors/MS580301BA07/MS580301BA07.h>
#include <sensors/analog/pressure/MPXHZ6130A/MPXHZ6130A.h>
#include <sensors/analog/pressure/honeywell/SSCDANN030PAA.h>
#include <sensors/analog/pressure/honeywell/SSCDRRN015PDA.h>

#include <iostream>
#include <sstream>

#include "PinHandler/PinHandler.h"

namespace PinHandlerTest
{
#include "../test-pinhandler.cpp"
}

// Pressure sensors mappings
constexpr ADS1118::ADS1118Mux NXP_1       = ADS1118::ADS1118Mux::MUX_AIN0_GND;
constexpr ADS1118::ADS1118Mux NXP_2       = ADS1118::ADS1118Mux::MUX_AIN3_GND;
constexpr ADS1118::ADS1118Mux HONEYWELL_1 = ADS1118::ADS1118Mux::MUX_AIN2_GND;
constexpr ADS1118::ADS1118Mux HONEYWELL_2 = ADS1118::ADS1118Mux::MUX_AIN1_GND;

// Sample frequency
constexpr int SAMPLING_FREQUENCY = 600;

// Voltage supplied to the analog sensors
constexpr int SUPPLIED_VOLTAGE = 5;  // Measure and change!

int menu();
int askSeconds();
template <typename PressureSensor>
void sampleAnalogPressureSensor(ADS1118::ADS1118Mux channel);
void sampleMS5803();
void sampleAll();

int main()
{
    TimestampTimer::enableTimestampTimer();

    switch (menu())
    {
        case 1:
            sampleAnalogPressureSensor<MPXHZ6130A>(NXP_1);
            break;
        case 2:
            sampleAnalogPressureSensor<MPXHZ6130A>(NXP_2);
            break;
        case 3:
            sampleAnalogPressureSensor<SSCDANN030PAA>(HONEYWELL_1);
            break;
        case 4:
            sampleAnalogPressureSensor<SSCDRRN015PDA>(HONEYWELL_2);
            break;
        case 5:
            sampleMS5803();
            break;
        case 6:
            sampleAll();
            break;
        case 7:
            PinHandlerTest::main();
            break;

        default:
            break;
    }

    return 0;
}

int menu()
{
    int choice;

    printf("\n\nWhat do you want to do?\n");
    printf("1. Sample NXP 1 (absolute)\n");
    printf("2. Sample NXP 2 (not installed)\n");
    printf("3. Sample HONEYWELL 1 (absolute)\n");
    printf("4. Sample HONEYWELL 2 (differential)\n");
    printf("5. Sample MS580301BA07\n");
    printf("6. Sample all the above\n");
    printf("7. Test detachment pins\n");
    printf("\n>> ");
    scanf("%d", &choice);

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

template <typename PressureSensor>
void sampleAnalogPressureSensor(ADS1118::ADS1118Mux channel)
{
    // Ask the user how many second the test should be perfomed
    int seconds = askSeconds();

    // Sensor setup

    SPIBus spiBus(SPI1);
    SPIBusConfig spiConfig = ADS1118::getDefaultSPIConfig();
    spiConfig.clock_div    = SPIClockDivider::DIV64;
    SPISlave spiSlave(spiBus, miosix::sensors::ads1118::cs::getPin(),
                      spiConfig);

    ADS1118::ADS1118Config ads1118Config = ADS1118::ADS1118_DEFAULT_CONFIG;
    ads1118Config.bits.mode = ADS1118::ADS1118Mode::CONTIN_CONV_MODE;
    ADS1118 ads1118         = ADS1118(spiSlave, ads1118Config);
    ads1118.init();
    ads1118.enableInput(channel, ADS1118::ADS1118DataRate::DR_860,
                        ADS1118::ADS1118Pga::FSR_6_144);

    std::function<ADCData()> get_voltage_function =
        std::bind(&ADS1118::getVoltage, &ads1118, channel);

    PressureSensor pressureSensor(get_voltage_function, SUPPLIED_VOLTAGE);

    // Sampling
    for (int i = 0; i < seconds * SAMPLING_FREQUENCY; i++)
    {
        ads1118.sample();
        pressureSensor.sample();
        printf("%llu,%.2f\n", pressureSensor.getLastSample().press_timestamp,
               pressureSensor.getLastSample().press);

        miosix::delayMs(1000 / SAMPLING_FREQUENCY);
    }
}

void sampleMS5803()
{
    // Ask the user how many second the test should be perfomed
    int seconds = askSeconds();

    // Sensor setup

    SPIBus spiBus(SPI1);

    MS580301BA07 ms5803 =
        MS580301BA07(spiBus, miosix::sensors::ms5803::cs::getPin());
    ms5803.init();

    // Sampling
    for (int i = 0; i < seconds * SAMPLING_FREQUENCY; i++)
    {
        ms5803.sample();
        printf("%.2f\n", ms5803.getLastSample().press);

        miosix::delayMs(1000 / SAMPLING_FREQUENCY);
    }
}

void sampleAll()
{
    // Ask the user how many second the test should be perfomed
    int seconds = askSeconds();

    // Sensor setup

    SPIBus spiBus(SPI1);
    SPIBusConfig spiConfig = ADS1118::getDefaultSPIConfig();
    spiConfig.clock_div    = SPIClockDivider::DIV64;
    SPISlave spiSlave(spiBus, miosix::sensors::ads1118::cs::getPin(),
                      spiConfig);

    ADS1118::ADS1118Config ads1118Config = ADS1118::ADS1118_DEFAULT_CONFIG;
    ads1118Config.bits.mode = ADS1118::ADS1118Mode::CONTIN_CONV_MODE;
    ADS1118 ads1118         = ADS1118(spiSlave, ads1118Config);
    ads1118.init();
    ads1118.enableInput(NXP_1, ADS1118::ADS1118DataRate::DR_860,
                        ADS1118::ADS1118Pga::FSR_6_144);
    ads1118.enableInput(NXP_2, ADS1118::ADS1118DataRate::DR_860,
                        ADS1118::ADS1118Pga::FSR_6_144);
    ads1118.enableInput(HONEYWELL_1, ADS1118::ADS1118DataRate::DR_860,
                        ADS1118::ADS1118Pga::FSR_6_144);
    ads1118.enableInput(HONEYWELL_2, ADS1118::ADS1118DataRate::DR_860,
                        ADS1118::ADS1118Pga::FSR_6_144);

    std::function<ADCData()> get_voltage_function_nxp_1 =
        std::bind(&ADS1118::getVoltage, &ads1118, NXP_1);
    std::function<ADCData()> get_voltage_function_nxp_2 =
        std::bind(&ADS1118::getVoltage, &ads1118, NXP_2);
    std::function<ADCData()> get_voltage_function_honeywell_1 =
        std::bind(&ADS1118::getVoltage, &ads1118, HONEYWELL_1);
    std::function<ADCData()> get_voltage_function_honeywell_2 =
        std::bind(&ADS1118::getVoltage, &ads1118, HONEYWELL_2);

    MPXHZ6130A npx1(get_voltage_function_nxp_1, SUPPLIED_VOLTAGE);
    MPXHZ6130A npx2(get_voltage_function_nxp_2, SUPPLIED_VOLTAGE);
    SSCDANN030PAA honeywell1(get_voltage_function_honeywell_1,
                             SUPPLIED_VOLTAGE);
    SSCDRRN015PDA honeywell2(get_voltage_function_honeywell_2,
                             SUPPLIED_VOLTAGE);
    MS580301BA07 ms5803 =
        MS580301BA07(spiBus, miosix::sensors::ms5803::cs::getPin());
    ms5803.init();

    // Sampling
    for (int i = 0; i < seconds * SAMPLING_FREQUENCY; i++)
    {
        ads1118.sample();
        npx1.sample();
        npx2.sample();
        honeywell1.sample();
        honeywell2.sample();
        ms5803.sample();
        printf("%llu,%.2f,%.2f,%.2f,%.2f,%.2f\n",
               ads1118.getLastSample().adc_timestamp,
               npx1.getLastSample().press, npx2.getLastSample().press,
               honeywell1.getLastSample().press,
               honeywell2.getLastSample().press, ms5803.getLastSample().press);

        miosix::delayMs(1000 / SAMPLING_FREQUENCY);
    }
}
