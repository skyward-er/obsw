/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Riccardo Musso
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

#pragma once

#include <configs/SensorManagerConfig.h>
#include <sensors/BMX160/BMX160.h>
#include <sensors/Sensor.h>
#include <sensors/calibration/Calibration.h>
#include <sensors/calibration/SixParameterCalibration.h>
#include <sensors/calibration/SoftIronCalibration.h>

#include <fstream>

struct BMX160CorrectionParameters
{
    Matrix<float, 3, 2> accelParams, magnetoParams, gyroParams;

    static std::string header()
    {
        return "accel_p1,accel_p2,accel_p3,accel_q1,accel_q2,accel_q3,"
               "mag_p1,mag_p2,mag_p3,mag_q1,mag_q2,mag_q3,"
               "gyro_p1,gyro_p2,gyro_p3,gyro_q1,gyro_q2,gyro_q3";
    }

    void read(std::istream& is)
    {
        for (int which = 0; which < 3; which++)
        {
            Matrix<float, 3, 2>& mat =
                (which == 0) ? accelParams
                             : (which == 1) ? magnetoParams : gyroParams;

            for(int i = 0; i < 2; i++){
                for(int j = 0; j < 3; j++){
                    is >> mat(j, i);

                    if (i != 1 || j != 2 || which != 2)
                        is.ignore(1, ',');

                }
            }
        }
    }

    void print(std::ostream& os) const
    {
        os << accelParams(0, 0) << "," << accelParams(1, 0) << ","
           << accelParams(2, 0) << "," << accelParams(0, 1) << ","
           << accelParams(1, 1) << "," << accelParams(2, 1) << ",";

        os << magnetoParams(0, 0) << "," << magnetoParams(1, 0) << ","
           << magnetoParams(2, 0) << "," << magnetoParams(0, 1) << ","
           << magnetoParams(1, 1) << "," << magnetoParams(2, 1) << ",";

        os << gyroParams(0, 0) << "," << gyroParams(1, 0) << ","
           << gyroParams(2, 0) << "," << gyroParams(0, 1) << ","
           << gyroParams(1, 1) << "," << gyroParams(2, 1);
    }
};

class BMX160Corrector : public Sensor<BMX160Data>
{
public:
    BMX160Corrector(BMX160* _driver) : driver(_driver) {}

    BMX160Corrector() : driver(NULL) {}

    void setDriver(BMX160* _driver) { driver = _driver; }

    BMX160* getDriver() { return driver; }

    bool init() override { return driver->init(); }

    bool selfTest() override { return driver->selfTest(); }

    BMX160Data sampleImpl() override
    {
        if (!driver)
        {
            TRACE("Error: driver doesn't point to valid sensor.");
            return {};
        }

        BMX160Data avg   = {}, fifoElem;
        uint8_t fifoSize, numAccel, numMag, numGyro;
        uint64_t accelTimestamp, magTimestamp, gyroTimestamp;

        std::array<BMX160Data, 200> fifo = driver->getLastFifo(); 
        fifoSize = driver->getLastFifoSize();

        numAccel = numMag = numGyro = 0;
        accelTimestamp = magTimestamp = gyroTimestamp = 0;

        // Get average values for measurements on each axis
        for (int i = 0; i < fifoSize; i++)
        {
            if(fifo[i].accel_timestamp != accelTimestamp){
                avg.accel_x += fifo[i].accel_x;
                avg.accel_y += fifo[i].accel_y;
                avg.accel_z += fifo[i].accel_z;

                accelTimestamp = fifo[i].accel_timestamp;
                numAccel++;
            }

            if(fifo[i].mag_timestamp != magTimestamp){
                avg.mag_x += fifo[i].mag_x;
                avg.mag_y += fifo[i].mag_y;
                avg.mag_z += fifo[i].mag_z;

                magTimestamp = fifo[i].mag_timestamp;
                numMag++;
            }
            
            if(fifo[i].gyro_timestamp != gyroTimestamp){
                avg.gyro_x += fifo[i].gyro_x;
                avg.gyro_y += fifo[i].gyro_y;
                avg.gyro_z += fifo[i].gyro_z;

                gyroTimestamp = fifo[i].gyro_timestamp;
                numGyro++;
            }
        }

        if(numAccel == 0){
            avg.accel_x = last_sample.accel_x;
            avg.accel_y = last_sample.accel_y;
            avg.accel_z = last_sample.accel_z;
        } else {
            avg.accel_x /= numAccel;
            avg.accel_y /= numAccel;
            avg.accel_z /= numAccel;
        }

        if(numMag == 0){
            avg.mag_x = last_sample.mag_x;
            avg.mag_y = last_sample.mag_y;
            avg.mag_z = last_sample.mag_z;
        } else {
            avg.mag_x /= numMag;
            avg.mag_y /= numMag;
            avg.mag_z /= numMag;
        }

        if(numGyro == 0){
            avg.gyro_x = last_sample.gyro_x;
            avg.gyro_y = last_sample.gyro_y;
            avg.gyro_z = last_sample.gyro_z;
        } else {
            avg.gyro_x /= numGyro;
            avg.gyro_y /= numGyro;
            avg.gyro_z /= numGyro;
        }

        TRACE("Number of accelerometer samples considered: %d\n", numAccel);
        TRACE("Number of magnetometer samples considered: %d\n", numMag);
        TRACE("Number of gyroscope samples considered: %d\n", numGyro);

        // Correct the average measurements
        auto acc    = accelCorrector.correct(avg);
        avg.accel_x = acc.accel_x;
        avg.accel_y = acc.accel_y;
        avg.accel_z = acc.accel_z;

        auto magneto = magnetoCorrector.correct(avg);
        avg.mag_x    = magneto.mag_x;
        avg.mag_y    = magneto.mag_y;
        avg.mag_z    = magneto.mag_z;

        auto gyro  = gyroCorrector.correct(avg);
        avg.gyro_x = gyro.gyro_x;
        avg.gyro_y = gyro.gyro_y;
        avg.gyro_z = gyro.gyro_z;

        uint64_t timestamp = TimestampTimer::getTimestamp(); 

        avg.accel_timestamp = timestamp;
        avg.mag_timestamp = timestamp;
        avg.gyro_timestamp = timestamp;

        return avg;
    }

    void setParameters(const BMX160CorrectionParameters& params)
    {
        accelCorrector << params.accelParams;
        magnetoCorrector << params.magnetoParams;
        gyroCorrector << params.gyroParams;
    }

    BMX160CorrectionParameters getParameters()
    {
        BMX160CorrectionParameters params;

        accelCorrector >> params.accelParams;
        magnetoCorrector >> params.magnetoParams;
        gyroCorrector >> params.gyroParams;

        return params;
    }

    void readParametersFromFile()
    {
        BMX160CorrectionParameters params;
        std::ifstream input(DeathStackBoard::Bmx160CorrectionParametersFile);

        // ignore first line (csv header)
        input.ignore(1000, '\n');

        params.read(input);
        setParameters(params);
    }

private:
    BMX160* driver;

    SixParameterCorrector<AccelerometerData> accelCorrector;
    SixParameterCorrector<MagnetometerData> magnetoCorrector;
    SixParameterCorrector<GyroscopeData> gyroCorrector;
};

