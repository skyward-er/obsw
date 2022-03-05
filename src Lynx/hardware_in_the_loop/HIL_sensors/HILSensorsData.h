/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <sensors/SensorData.h>

struct HILAccelData : public AccelerometerData
{
    HILAccelData() : AccelerometerData{0, 0.0, 0.0, 0.0} {}

    HILAccelData(uint64_t t, float x, float y, float z)
        : AccelerometerData{t, x, y, z}
    {
    }

    static std::string header() { return "timestamp,x,y,z\n"; }

    void print(std::ostream& os) const
    {
        os << accel_timestamp << "," << accel_x << "," << accel_y << ","
           << accel_z << "\n";
    }
};

struct HILGyroscopeData : public GyroscopeData
{
    HILGyroscopeData() : GyroscopeData{0, 0.0, 0.0, 0.0} {}

    HILGyroscopeData(uint64_t t, float x, float y, float z)
        : GyroscopeData{t, x, y, z}
    {
    }

    static std::string header() { return "timestamp,x,y,z\n"; }

    void print(std::ostream& os) const
    {
        os << gyro_timestamp << "," << gyro_x << "," << gyro_y << "," << gyro_z
           << "\n";
    }
};

struct HILMagnetometerData : public MagnetometerData
{
    HILMagnetometerData() : MagnetometerData{0, 0.0, 0.0, 0.0} {}

    HILMagnetometerData(uint64_t t, float x, float y, float z)
        : MagnetometerData{t, x, y, z}
    {
    }

    static std::string header() { return "timestamp,x,y,z\n"; }

    void print(std::ostream& os) const
    {
        os << mag_timestamp << "," << mag_x << "," << mag_y << "," << mag_z
           << "\n";
    }
};

struct HILImuData : public HILAccelData,
                    public HILGyroscopeData,
                    public HILMagnetometerData
{
    static std::string header()
    {
        return "accel_timestamp,accel_x,accel_y,accel_z,gyro_timestamp,gyro_x,"
               "gyro_y,gyro_z,mag_timestamp,mag_x,mag_y,mag_z\n";
    }

    void print(std::ostream& os) const
    {
        os << accel_timestamp << "," << accel_x << "," << accel_y << ","
           << accel_z << "," << gyro_timestamp << "," << gyro_x << "," << gyro_y
           << "," << gyro_z << "," << mag_timestamp << "," << mag_x << ","
           << mag_y << "," << mag_z << "\n";
    }
};

struct HILGpsData : public GPSData
{
    HILGpsData() : GPSData{0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3, true}
    {
    }

    HILGpsData(uint64_t t, float x, float y, float z, float v_x, float v_y,
               float v_z, float v)
        : GPSData{t, x, y, z, v_x, v_y, v_z, v, 0.0, 3, true}
    {
    }

    static std::string header()
    {
        return "timestamp,x,y,z,v_x,v_y,v_z,v,track,n_sats,fix\n";
    }

    void print(std::ostream& os) const
    {
        os << gps_timestamp << "," << longitude << "," << latitude << ","
           << height << "," << velocity_north << "," << velocity_east << ","
           << velocity_down << "," << speed << "," << track << ","
           << (int)num_satellites << "," << (int)fix << "\n";
    }
};

struct HILBaroData : public PressureData
{
    HILBaroData() : PressureData{0, 0.0} {}

    HILBaroData(uint64_t t, float p) : PressureData{t, p} {}

    static std::string header() { return "timestamp,press\n"; }

    void print(std::ostream& os) const
    {
        os << press_timestamp << "," << press << "\n";
    }
};
