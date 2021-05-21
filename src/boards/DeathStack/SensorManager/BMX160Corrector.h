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
#include <sensors/calibration/SensorDataExtra.h>
#include <sensors/calibration/SixParameterCalibration.h>
#include <sensors/calibration/SoftIronCalibration.h>

#include <fstream>

struct BMX160DataCorrected : public BMX160Data
{
    BMX160DataCorrected() : BMX160Data() {}

    BMX160DataCorrected(const BMX160Data& data) 
        : BMX160DataCorrected(data, data, data){}

    BMX160DataCorrected(AccelerometerData acc, GyroscopeData gyr,
                        MagnetometerData mag)
        : BMX160Data(acc, gyr, mag)
    {
    }
};

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

            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 3; j++)
                {
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

class BMX160Corrector : public Sensor<BMX160DataCorrected>
{
public:
    BMX160Corrector(BMX160* _driver) : driver(_driver) {}

    BMX160Corrector() : driver(nullptr) {}

    void setDriver(BMX160* _driver) { driver = _driver; }

    BMX160* getDriver() { return driver; }

    bool init() override { return true; }

    bool selfTest() override { return true; }

    static void setAccel(BMX160Data& lhs, const BMX160Data& rhs)
    {
        lhs.accel_x = rhs.accel_x;
        lhs.accel_y = rhs.accel_y;
        lhs.accel_x = rhs.accel_x;
    }

    void calibrate()
    {
        // readParametersFromFile();

        Matrix<float, 3, 2> mat;
        mat.col(0) = Vector3f({2, 2, 2}).transpose();
        mat.col(1) = Vector3f({0, 0, 0}).transpose();

        BMX160CorrectionParameters params;
        params.accelParams   = mat;
        params.magnetoParams = mat;
        params.gyroParams    = mat;

        setParameters(params);
    }

    static BMX160DataCorrected rotateAxis(BMX160DataCorrected data)
    {
        // TODO : use rotaton matrix
        BMX160DataCorrected temp;
        temp.accel_timestamp = data.accel_timestamp;
        temp.gyro_timestamp  = data.gyro_timestamp;
        temp.mag_timestamp   = data.mag_timestamp;

        // sensor's Z is X in body frame
        temp.accel_z = data.accel_x;
        temp.gyro_z  = data.gyro_x;
        temp.mag_z   = data.mag_x;

        // sensor's X is -Z in body frame
        temp.accel_x = -data.accel_z;
        temp.gyro_x  = -data.gyro_z;
        temp.mag_x   = -data.mag_z;

        // sensor's Y is -Y in body frame
        temp.accel_y = -data.accel_y;
        temp.gyro_y  = -data.gyro_y;
        temp.mag_y   = -data.mag_y;

        return temp;
    }

private:
    BMX160DataCorrected sampleImpl() override
    {
        if (!driver)
        {
            TRACE("Error: driver doesn't point to valid sensor.");
            return BMX160DataCorrected{};
        }

        Vector3f avgAccel{0, 0, 0}, avgMag{0, 0, 0}, avgGyro{0, 0, 0},
            vec{0, 0, 0};
        uint64_t accelTimestamp = 0, magTimestamp = 0, gyroTimestamp = 0;
        uint8_t numAccel = 0, numMag = 0, numGyro = 0, fifoSize = 0;
        BMX160Data fifoElem;
        BMX160DataCorrected res;

        fifoSize = driver->getLastFifoSize();

        // Get average values for measurements on each axis
        for (int i = 0; i < fifoSize; i++)
        {
            fifoElem = driver->getFifoElement(i);

            if (fifoElem.accel_timestamp > accelTimestamp)
            {
                static_cast<AccelerometerData>(fifoElem) >> vec;
                avgAccel += vec;

                accelTimestamp = fifoElem.accel_timestamp;
                numAccel++;
            }

            if (fifoElem.mag_timestamp > magTimestamp)
            {
                static_cast<MagnetometerData>(fifoElem) >> vec;
                avgMag += vec;

                magTimestamp = fifoElem.mag_timestamp;
                numMag++;
            }

            if (fifoElem.gyro_timestamp > gyroTimestamp)
            {
                static_cast<GyroscopeData>(fifoElem) >> vec;
                avgGyro += vec;

                gyroTimestamp = fifoElem.gyro_timestamp;
                numGyro++;
            }
        }

        if (numAccel == 0)
            static_cast<AccelerometerData>(driver->last_sample) >> avgAccel;
        else
            avgAccel /= numAccel;

        if (numMag == 0)
            static_cast<MagnetometerData>(driver->last_sample) >> avgMag;
        else
            avgMag /= numMag;

        if (numGyro == 0)
            static_cast<GyroscopeData>(driver->last_sample) >> avgGyro;
        else
            avgGyro /= numGyro;

        //TRACE("Number of accelerometer samples considered: %d\n", numAccel);
        //TRACE("Number of magnetometer samples considered: %d\n", numMag);
        //TRACE("Number of gyroscope samples considered: %d\n", numGyro);

        static_cast<AccelerometerData&>(res) << avgAccel;
        static_cast<MagnetometerData&>(res) << avgMag;
        static_cast<GyroscopeData&>(res) << avgGyro;

        // Correct the average measurements
        auto acc = accelCorrector.correct(res);
        setAccelCorrected(res, acc);

        auto mag = magnetoCorrector.correct(res);
        setMagCorrected(res, mag);

        auto gyro = gyroCorrector.correct(res);
        setGyroCorrected(res, gyro);

        // get the timestamp of the newest value in fifo
        uint64_t timestamp =
            fifoElem.accel_timestamp;  // TimestampTimer::getTimestamp();
        res.accel_timestamp = timestamp;
        res.mag_timestamp   = timestamp;
        res.gyro_timestamp  = timestamp;

        //res = rotateAxis(res);

        return res;
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

    void setAccelCorrected(BMX160DataCorrected& lhs,
                           const AccelerometerData& rhs)
    {
        lhs.accel_x = rhs.accel_x;
        lhs.accel_y = rhs.accel_y;
        lhs.accel_z = rhs.accel_z;
    }

    void setGyroCorrected(BMX160DataCorrected& lhs, const GyroscopeData& rhs)
    {
        lhs.gyro_x = rhs.gyro_x;
        lhs.gyro_y = rhs.gyro_y;
        lhs.gyro_z = rhs.gyro_z;
    }

    void setMagCorrected(BMX160DataCorrected& lhs, const MagnetometerData& rhs)
    {
        lhs.mag_x = rhs.mag_x;
        lhs.mag_y = rhs.mag_y;
        lhs.mag_z = rhs.mag_z;
    }

    BMX160* driver;

    SixParameterCorrector<AccelerometerData> accelCorrector;
    SixParameterCorrector<MagnetometerData> magnetoCorrector;
    SixParameterCorrector<GyroscopeData> gyroCorrector;
};
