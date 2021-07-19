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

#include "BMX160Calibrator.h"

BMX160Calibrator::BMX160Calibrator(BMX160* _driver) : driver(_driver) {}

BMX160Calibrator::BMX160Calibrator() : driver(nullptr) {}

void BMX160Calibrator::setDriver(BMX160* _driver) { driver = _driver; }

BMX160* BMX160Calibrator::getDriver() { return driver; }

bool BMX160Calibrator::init() { return true; }

bool BMX160Calibrator::selfTest() { return true; }

bool BMX160Calibrator::calibrate()
{

    // Matrix<float, 3, 2> mat;
    // mat.col(0) = Vector3f({2, 2, 2}).transpose();
    // mat.col(1) = Vector3f({0, 0, 0}).transpose();

    // BMX160CorrectionParameters params;
    // params.accelParams   = mat;
    // params.magnetoParams = mat;
    // params.gyroParams    = mat;

    // setParameters(params);

    // return a boolean to indicate if calibration has ended
    // some other component will poll this method to know when
    // calibration is done
    
    /*TRACE("Called calibrate()\n");
    TRACE("samples_num: %d\n", samples_num);*/

    if (!is_calibrating)
    {
        readParametersFromFile();
        is_calibrating = true;
        samples_num    = 0;

        gyroCalibrator = BiasCalibration<GyroscopeData>();
        gyroCalibrator.setReferenceVector(
            {0, 0, 0});  // gyro has to read 0 on all axis while stopped
    }
    else
    {
        // calibration is done, replace the offsets in gyroCalibrator
        // with the computed ones

        // also store offsets in a struct of type BMX160GyroOffsets

        if (samples_num >= 1)
        {
            is_calibrating = false;
            gyroCorrector  = gyroCalibrator.computeResult();

            Vector3f params;
            gyroCorrector >> params;
            //TRACE("Params: %f, %f, %f\n", params(0), params(1), params(2));
        }
    }

    return is_calibrating;
}

BMX160DataCorrected BMX160Calibrator::rotateAxis(BMX160DataCorrected data)
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

BMX160DataCorrected BMX160Calibrator::sampleImpl()
{
    if (!driver)
    {
        TRACE("Error: driver doesn't point to valid sensor.");
        return BMX160DataCorrected{};
    }

    Vector3f avgAccel{0, 0, 0}, avgMag{0, 0, 0}, avgGyro{0, 0, 0}, vec{0, 0, 0};
    uint64_t accelTimestamp = 0, magTimestamp = 0, gyroTimestamp = 0;
    uint8_t numAccel = 0, numMag = 0, numGyro = 0, fifoSize = 0;
    BMX160Data fifoElem;
    BMX160DataCorrected res;

    fifoSize = driver->getLastFifoSize();

    // Get average values for measurements on each axis
    for (int i = 0; i < fifoSize; i++)
    {
        fifoElem = driver->getFifoElement(i);

        // if calibration is ongoing, simply feed the model
        // with the sampled data

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
            if (is_calibrating)
            {
                /*TRACE("Feeding with %f, %f, %f\n", fifoElem.gyro_x,
                       fifoElem.gyro_y, fifoElem.gyro_z);*/
                gyroCalibrator.feed(fifoElem);
                samples_num++;
            }

            static_cast<GyroscopeData>(fifoElem) >> vec;
            avgGyro += vec;

            gyroTimestamp = fifoElem.gyro_timestamp;
            numGyro++;
        }
    }

    if (numAccel == 0)
        static_cast<AccelerometerData>(driver->getLastSample()) >> avgAccel;
    else
        avgAccel /= numAccel;

    if (numMag == 0)
        static_cast<MagnetometerData>(driver->getLastSample()) >> avgMag;
    else
        avgMag /= numMag;

    if (numGyro == 0)
        static_cast<GyroscopeData>(driver->getLastSample()) >> avgGyro;
    else
        avgGyro /= numGyro;

    // TRACE("Number of accelerometer samples considered: %d\n", numAccel);
    // TRACE("Number of magnetometer samples considered: %d\n", numMag);
    // TRACE("Number of gyroscope samples considered: %d\n", numGyro);

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

    Vector3f gyro_offsets;
    gyroCorrector >> gyro_offsets;
    /*TRACE("Gyro params: %f, %f, %f\n", gyro_offsets(0), gyro_offsets(1),
           gyro_offsets(2));
    TRACE("Corrected: %f, %f, %f\n", res.gyro_x, res.gyro_y, res.gyro_z);*/
    res = rotateAxis(res);

    return res;
}

void BMX160Calibrator::setParameters(const BMX160CorrectionParameters& params)
{
    accelCorrector << params.accelParams;
    magnetoCorrector << params.magnetoParams;
    gyroCorrector << params.gyroParams;
}

BMX160CorrectionParameters BMX160Calibrator::getParameters()
{
    BMX160CorrectionParameters params;

    accelCorrector >> params.accelParams;
    magnetoCorrector >> params.magnetoParams;
    gyroCorrector >> params.gyroParams;

    return params;
}

void BMX160Calibrator::readParametersFromFile()
{
    BMX160CorrectionParameters params;
    std::ifstream input(
        DeathStackBoard::SensorConfigs::Bmx160CorrectionParametersFile);

    // ignore first line (csv header)
    input.ignore(1000, '\n');

    params.read(input);
    setParameters(params);
}

void BMX160Calibrator::setAccelCorrected(BMX160DataCorrected& lhs,
                                         const AccelerometerData& rhs)
{
    lhs.accel_x = rhs.accel_x;
    lhs.accel_y = rhs.accel_y;
    lhs.accel_z = rhs.accel_z;
}

void BMX160Calibrator::setGyroCorrected(BMX160DataCorrected& lhs,
                                        const GyroscopeData& rhs)
{
    lhs.gyro_x = rhs.gyro_x;
    lhs.gyro_y = rhs.gyro_y;
    lhs.gyro_z = rhs.gyro_z;
}

void BMX160Calibrator::setMagCorrected(BMX160DataCorrected& lhs,
                                       const MagnetometerData& rhs)
{
    lhs.mag_x = rhs.mag_x;
    lhs.mag_y = rhs.mag_y;
    lhs.mag_z = rhs.mag_z;
}
