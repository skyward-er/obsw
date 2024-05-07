/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

#include <drivers/spi/SPIDriver.h>
#include <miosix.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSpi.h>
#include <utils/Debug.h>

using namespace Boardcore;
using namespace miosix;

GpioPin clk(GPIOA_BASE, 5);
GpioPin miso(GPIOA_BASE, 6);
GpioPin mosi(GPIOA_BASE, 7);

SPIBus spiBus(SPI1);

void initSPI();

int testGPS(GpioPin cs);
int testLPS(GpioPin cs);
int testH3LIS(GpioPin cs);
int testADS(GpioPin cs);

int main()
{

    GpioPin lps22CS(GPIOB_BASE, 1);
    GpioPin lps22DevBoardCS(GPIOG_BASE, 7);
    GpioPin h3lisCS(GPIOC_BASE, 6);
    GpioPin ads131CS(GPIOC_BASE, 2);
    GpioPin ubxgpsCS(GPIOD_BASE, 7);

    // disable cs
    lps22CS.mode(miosix::Mode::OUTPUT);
    lps22CS.high();

    lps22DevBoardCS.mode(miosix::Mode::OUTPUT);
    lps22DevBoardCS.high();

    lps22CS.mode(miosix::Mode::OUTPUT);
    lps22CS.high();

    h3lisCS.mode(miosix::Mode::OUTPUT);
    h3lisCS.high();

    ads131CS.mode(miosix::Mode::OUTPUT);
    ads131CS.high();

    ubxgpsCS.mode(miosix::Mode::OUTPUT);
    ubxgpsCS.high();

    initSPI();

    // testLPS(lps22CS);

    // testLPS(lps22DevBoardCS);

    // testH3LIS(h3lisCS);

    testADS(ads131CS);

    // testGPS(ubxgpsCS);
}
void initSPI()
{
    // Setup gpio pins
    clk.mode(Mode::ALTERNATE);
    clk.alternateFunction(5);
    miso.mode(Mode::ALTERNATE);
    miso.alternateFunction(5);
    mosi.mode(Mode::ALTERNATE);
    mosi.alternateFunction(5);
}

int testGPS(GpioPin cs)
{
    TRACE("Initializing UBXGPSSpi...\n");
    UBXGPSSpi gps{spiBus, cs};

    while (!gps.init())
    {
        TRACE("Init failed! (code: %d)\n", gps.getLastError());

        TRACE("Retrying in 10 seconds...\n");
        miosix::Thread::sleep(10000);
    }

    while (true)
    {
        gps.sample();
        GPSData sample __attribute__((unused)) = gps.getLastSample();

        TRACE(
            "timestamp: %4.3f, lat: %f, lon: %f, height: %4.1f, velN: %3.2f, "
            "velE: %3.2f, velD: %3.2f, speed: %3.2f, track %3.1f, pDOP: %f, "
            "nsat: %2d, fix: %2d\n",
            (float)sample.gpsTimestamp / 1000000, sample.latitude,
            sample.longitude, sample.height, sample.velocityNorth,
            sample.velocityEast, sample.velocityDown, sample.speed,
            sample.track, sample.positionDOP, sample.satellites, sample.fix);

        Thread::sleep(1000);
    }
    return 0;
}

int testLPS(GpioPin cs)
{
    LPS22DF::Config config;
    config.odr = Boardcore::LPS22DF::ODR_100;
    config.avg = Boardcore::LPS22DF::AVG_4;

    auto lpsConfig = LPS22DF::getDefaultSPIConfig();

    lpsConfig.clockDivider = SPI::ClockDivider::DIV_64;

    LPS22DF sensor(spiBus, cs, lpsConfig, config);

    printf("Starting...\n");

    if (!sensor.init())
    {
        printf("Init failed\n");
        miosix::Thread::sleep(10000);
        return -1;
    }

    if (!sensor.selfTest())
    {
        printf("Error: selfTest() returned false!\n");
        miosix::Thread::sleep(10000);
        return -1;
    }

    // printf("Trying one shot mode for 10 seconds\n");
    // for (int i = 0; i < 10 * 10; i++)
    // {
    //     sensor.sample();
    //     LPS22DFData data = sensor.getLastSample();

    //     printf("%.2f C | %.2f Pa\n", data.temperature, data.pressure);

    //     miosix::Thread::sleep(10000);
    // }

    printf("Now setting 10Hz continuous mode\n");
    sensor.setOutputDataRate(LPS22DF::ODR_10);
    int i = 10;
    while (i--)
    {
        sensor.sample();
        LPS22DFData data = sensor.getLastSample();

        printf("%.2f C | %.2f Pa\n", data.temperature, data.pressure);

        miosix::Thread::sleep(100);
    }
    return 0;
}

int testH3LIS(GpioPin cs)
{

    H3LIS331DL sensor(spiBus, cs, H3LIS331DLDefs::OutputDataRate::ODR_50,
                      H3LIS331DLDefs::BlockDataUpdate::BDU_CONTINUOS_UPDATE,
                      H3LIS331DLDefs::FullScaleRange::FS_100);

    if (!sensor.init())
    {
        printf("Failed init!\n");
        if (sensor.getLastError() == SensorErrors::INVALID_WHOAMI)
        {
            printf("Invalid WHOAMI\n");
        }
        Thread::sleep(5000);
        return -1;
    }

    H3LIS331DLData data;

    // Print out the CSV header
    printf(H3LIS331DLData::header().c_str());
    // sample some data from the sensor
    for (int i = 0; i < 30; i++)
    {
        // sensor intitialized, should return error if no new data exist
        sensor.sample();

        // if (sensor.getLastError() == SensorErrors::NO_NEW_DATA)
        // {
        //     printf("\nWarning: no new data to be read \n");
        // }

        data = sensor.getLastSample();

        printf("%llu,%f,%f,%f\n", data.accelerationTimestamp,
               data.accelerationX, data.accelerationY, data.accelerationZ);

        Thread::sleep(100);
    }

    return 0;
}

int testADS(GpioPin cs)
{
    // ADC configuration
    ADS131M08::Config config{
        .oversamplingRatio     = ADS131M08Defs::OversamplingRatio::OSR_8192,
        .globalChopModeEnabled = true,
    };

    // Device initialization

    ADS131M08 ads131(spiBus, cs, {}, config);

    ads131.init();

    printf("Now performing self test...\n");
    if (ads131.selfTest())
    {
        printf("Self test succeeded\n");
    }
    else
    {
        printf("Self test failed!\n");
        return -1;
    }

    ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_0);
    // ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_1);
    // ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_2);
    // ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_3);
    // ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_4);
    // ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_5);
    // ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_6);
    // ads131.calibrateOffset(ADS131M08Defs::Channel::CHANNEL_7);

    int i = 10;
    while (true)
    {
        ads131.sample();

        ADS131M08Data data = ads131.getLastSample();

        printf(
            "% 2.8f\n",
            // \t% 2.8f\t% 2.8f\t% 2.8f\t% 2.8f\t% 2.8f\t% 2.8f\t% 2.8f\n",
            data.voltage[0]/*, data.voltage[1], data.voltage[2], data.voltage[3],
            data.voltage[4], data.voltage[5], data.voltage[6], data.voltage[7]);*/
        );
        delayMs(200);
    }

    return 0;
}
