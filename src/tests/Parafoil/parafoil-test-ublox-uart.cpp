/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Davide Bonomini, Davide Mor, Alberto Nidasio, Damiano Amatruda
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

#include <drivers/timer/TimestampTimer.h>
#include <sensors/UbloxGPS/UbloxGPS.h>
#include <utils/Debug.h>

using namespace miosix;
using namespace Boardcore;

#define RATE 4

int main()
{
    (void)TimestampTimer::getInstance();

    GpioPin tx(GPIOA_BASE, 2);
    GpioPin rx(GPIOA_BASE, 3);

    tx.mode(miosix::Mode::ALTERNATE);
    tx.alternateFunction(7);
    rx.mode(miosix::Mode::ALTERNATE);
    rx.alternateFunction(7);

    printf("Welcome to the ublox test\n");

    // Keep GPS baud rate at default for easier testing
    UbloxGPS gps(38400, RATE, 2, "gps", 38400);
    UbloxGPSData dataGPS;
    printf("Gps allocated\n");

    // Init the gps
    if (gps.init())
    {
        printf("Successful gps initialization\n");
    }
    else
    {
        printf("Failed gps initialization\n");
    }

    // Perform the selftest
    if (gps.selfTest())
    {
        printf("Successful gps selftest\n");
    }
    else
    {
        printf("Failed gps selftest\n");
    }

    // Start the gps thread
    gps.start();
    printf("Gps started\n");

    while (true)
    {
        // Give time to the thread
        Thread::sleep(1000 / RATE);

        // Sample
        gps.sample();
        dataGPS = gps.getLastSample();

        // Print out the latest sample
        TRACE(
            "[gps] timestamp: % 4.3f, fix: %01d lat: % f lon: % f "
            "height: %4.1f nsat: %2d speed: %3.2f velN: % 3.2f velE: % 3.2f "
            "track %3.1f\n",
            (float)dataGPS.gpsTimestamp / 1000000, dataGPS.fix,
            dataGPS.latitude, dataGPS.longitude, dataGPS.height,
            dataGPS.satellites, dataGPS.speed, dataGPS.velocityNorth,
            dataGPS.velocityEast, dataGPS.track);
    }
}
