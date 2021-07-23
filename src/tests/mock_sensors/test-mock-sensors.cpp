/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include "Sensors/Mock/MockSensors.h"

using namespace DeathStackBoard;

int main()
{
    TimestampTimer::enableTimestampTimer();

    MockGPS gps;
    MockPressureSensor p_sensor;
    MockIMU imu;
    TestSensor test_sensor;

    int i = 0;

    while (true)
    {

        if (i == 10)
        {
            gps.signalLiftoff();
            p_sensor.signalLiftoff();
            TRACE("---------- LIFTOFF !!! ----------\n\n");
        }

        gps.sample();
        GPSData gps_data = gps.getLastSample();
        TRACE("GPS SAMPLE: \n");
        TRACE("lat = %f \n", gps_data.latitude);
        TRACE("lon = %f \n\n", gps_data.longitude);

        p_sensor.sample();
        PressureData pressure = p_sensor.getLastSample();
        TRACE("PRESSURE SAMPLE: \n");
        TRACE("pressure = %f \n\n", pressure.press);

        imu.sample();
        MockIMUData imu_data = imu.getLastSample();
        TRACE("IMU SAMPLE: \n");
        TRACE("accel x = %f \n", imu_data.accel_x);
        TRACE("accel y = %f \n", imu_data.accel_y);
        TRACE("accel z = %f \n", imu_data.accel_z);
        TRACE("gyro x = %f \n", imu_data.gyro_x);
        TRACE("gyro y = %f \n", imu_data.gyro_y);
        TRACE("gyro z = %f \n", imu_data.gyro_z);
        TRACE("magneto x = %f \n", imu_data.mag_x);
        TRACE("magneto y = %f \n", imu_data.mag_y);
        TRACE("magneto z = %f \n\n", imu_data.mag_z);

        test_sensor.sample();
        TestData test_data = test_sensor.getLastSample();
        TRACE("TEST SAMPLE: \n");
        TRACE("value = %f \n\n", test_data.value);

        miosix::Thread::sleep(1000);

        i++;
    }
}