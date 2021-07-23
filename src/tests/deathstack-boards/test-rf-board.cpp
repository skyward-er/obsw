/* Copyright (c) 2021 Skyward Experimental Rocketry
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

/**
 * Components on the rf board:
 *
 * LIS3MDL: Magnetometer
 * ASM330LHH: Accelerometer and gyroscope
 * LSM9DS1: Accelerometer, gyroscope and magnetometer
 * BMX160: Accelerometer and gyroscope
 * GPS
 * SD card
 */

#include <Common.h>
#include <drivers/gps/ublox/UbloxGPS.h>
#include <drivers/spi/SPIDriver.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/LIS3MDL/LIS3MDL.h>

#include <ctime>
#include <iostream>
#include <sstream>
#include <vector>

#include "math/Stats.h"

namespace SDCardBenchmark
{
#include "../../skyward-boardcore/src/entrypoints/sdcard-benchmark.cpp"
}

// Sample frequency
constexpr int SAMPLING_FREQUENCY = 100;

int menu();
int askSeconds();
void sampleLIS3MDL();
// void sampleASM330LHH();
// void sampleLSM9DSI();
void sampleBMX160();
void sampleAll();
void sampleGPS();

int main()
{
    switch (menu())
    {
        case 1:
            sampleLIS3MDL();
            break;
        case 2:
            printf("Component not currently installed!\n");
            // sampleASM330LHH();
            break;
        case 3:
            printf("Component not currently installed!\n");
            // sampleLSM9DSI();
            break;
        case 4:
            sampleAll();
            break;
        case 5:
            sampleBMX160();
            break;
        case 6:
            SDCardBenchmark::main();
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
    printf("1. Sample LIS3MDL\n");
    printf("2. Sample ASM330LHH (not installed)\n");
    printf("3. Sample LSM9DSI (not installed)\n");
    printf("4. Sample all the above\n");
    printf("5. Sample BMX160 (gps)\n");
    printf("6. SD Card benchmark\n");
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

void sampleLIS3MDL()
{
    // Ask the user how many second the test should be perfomed
    int seconds = askSeconds();

    // Sensor setup

    SPIBus spiBus(SPI1);
    SPIBusConfig spiConfig;
    spiConfig.clock_div = SPIClockDivider::DIV32;
    spiConfig.mode      = SPIMode::MODE1;

    LIS3MDL::Config lis3mdlConfig;
    lis3mdlConfig.odr                = LIS3MDL::ODR_560_HZ;
    lis3mdlConfig.scale              = LIS3MDL::FS_16_GAUSS;
    lis3mdlConfig.temperatureDivider = 5;

    LIS3MDL lis3mdl(spiBus, miosix::sensors::lis3mdl::cs::getPin(), spiConfig,
                    lis3mdlConfig);
    lis3mdl.init();

    // Sampling
    std::cout << LIS3MDLData::header();
    for (int i = 0; i < seconds * SAMPLING_FREQUENCY; i++)
    {
        lis3mdl.sample();
        LIS3MDLData data = lis3mdl.getLastSample();

        data.print(std::cout);

        miosix::delayMs(1000 / SAMPLING_FREQUENCY);
    }
}

void sampleBMX160()
{
    // Ask the user how many second the test should be perfomed
    int seconds = askSeconds();

    // Sensor setup

    SPIBus spiBus(SPI1);

    BMX160Config bmx160Config;
    bmx160Config.fifo_mode    = BMX160Config::FifoMode::DISABLED;
    bmx160Config.fifo_int     = BMX160Config::FifoInt::PIN_INT1;
    bmx160Config.temp_divider = 1;

    BMX160 bmx160 =
        BMX160(spiBus, miosix::sensors::bmx160::cs::getPin(), bmx160Config);
    bmx160.init();

    // Sampling
    std::cout << BMX160Data::header();
    for (int i = 0; i < seconds * SAMPLING_FREQUENCY; i++)
    {
        bmx160.sample();
        BMX160Data data = bmx160.getLastSample();

        data.print(std::cout);

        miosix::delayMs(1000 / SAMPLING_FREQUENCY);
    }
}

void sampleAll()
{
    // Ask the user how many second the test should be perfomed
    int seconds = askSeconds();

    // Sensor setup

    SPIBus spiBus(SPI1);
    SPIBusConfig spiConfig;
    spiConfig.clock_div = SPIClockDivider::DIV32;
    spiConfig.mode      = SPIMode::MODE1;

    LIS3MDL::Config lis3mdlConfig;
    lis3mdlConfig.odr                = LIS3MDL::ODR_560_HZ;
    lis3mdlConfig.scale              = LIS3MDL::FS_16_GAUSS;
    lis3mdlConfig.temperatureDivider = 5;

    LIS3MDL lis3mdl(spiBus, miosix::sensors::lis3mdl::cs::getPin(), spiConfig,
                    lis3mdlConfig);
    lis3mdl.init();

    BMX160Config bmx160Config;
    bmx160Config.fifo_mode    = BMX160Config::FifoMode::DISABLED;
    bmx160Config.fifo_int     = BMX160Config::FifoInt::PIN_INT1;
    bmx160Config.temp_divider = 1;

    BMX160 bmx160 =
        BMX160(spiBus, miosix::sensors::bmx160::cs::getPin(), bmx160Config);
    bmx160.init();

    // Sampling
    for (int i = 0; i < seconds * SAMPLING_FREQUENCY; i++)
    {
        lis3mdl.sample();
        LIS3MDLData lis3mdlData = lis3mdl.getLastSample();
        bmx160.sample();
        BMX160Data bmx160Data = bmx160.getLastSample();

        lis3mdlData.print(std::cout);
        bmx160Data.print(std::cout);

        miosix::delayMs(1000 / SAMPLING_FREQUENCY);
    }
}

void sampleGPS()
{
    // Ask the user how many second the test should be perfomed
    int seconds = askSeconds();

    // Sensor setup

    UbloxGPS gps(38400);
    gps.init();
    miosix::delayMs(200);
    gps.start();
    // gps.sendSBASMessage();

    // Sampling
    std::cout << UbloxGPSData::header();
    for (int i = 0; i < seconds; i++)
    {
        gps.sample();
        UbloxGPSData data = gps.getLastSample();

        data.print(std::cout);

        miosix::delayMs(1000);
    }
}
