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
#include <BMX160DataCorrected.h>

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

class BMX160Calibrator : public Sensor<BMX160DataCorrected>
{
public:
    BMX160Calibrator(BMX160* _driver);

    BMX160Calibrator();

    void setDriver(BMX160* _driver);

    BMX160* getDriver();

    bool init() override;

    bool selfTest() override;

    static void setAccel(BMX160Data& lhs, const BMX160Data& rhs);

    bool calibrate();

    static BMX160DataCorrected rotateAxis(BMX160DataCorrected data);

private:
    BMX160DataCorrected sampleImpl() override;

    void setParameters(const BMX160CorrectionParameters& params);

    BMX160CorrectionParameters getParameters();

    void readParametersFromFile();

    void setAccelCorrected(BMX160DataCorrected& lhs,
                           const AccelerometerData& rhs);
    void setGyroCorrected(BMX160DataCorrected& lhs, const GyroscopeData& rhs);
    void setMagCorrected(BMX160DataCorrected& lhs, const MagnetometerData& rhs);

    BMX160* driver;

    bool is_calibrating = false;

    SixParameterCorrector<AccelerometerData> accelCorrector;
    SixParameterCorrector<MagnetometerData> magnetoCorrector;
    SixParameterCorrector<GyroscopeData> gyroCorrector;

    SixParameterCalibration<GyroscopeData, 100> gyroCalibrator;
};
