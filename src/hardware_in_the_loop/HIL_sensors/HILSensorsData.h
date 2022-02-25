
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
        os << accelerationTimestamp << "," << accelerationX << ","
           << accelerationY << "," << accelerationZ << "\n";
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
        os << angularVelocityTimestamp << "," << angularVelocityX << ","
           << angularVelocityY << "," << angularVelocityZ << "\n";
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
        os << magneticFieldTimestamp << "," << magneticFieldX << ","
           << magneticFieldY << "," << magneticFieldZ << "\n";
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
        os << accelerationTimestamp << "," << accelerationX << ","
           << accelerationY << "," << accelerationZ << ","
           << angularVelocityTimestamp << "," << angularVelocityX << ","
           << angularVelocityY << "," << angularVelocityZ << ","
           << magneticFieldTimestamp << "," << magneticFieldX << ","
           << magneticFieldY << "," << magneticFieldZ << "\n";
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
        os << gpsTimestamp << "," << longitude << "," << latitude << ","
           << height << "," << velocityNorth << "," << velocityEast << ","
           << velocityDown << "," << speed << "," << track << ","
           << (int)satellites << "," << (int)fix << "\n";
    }
};

struct HILBaroData : public PressureData
{
    HILBaroData() : PressureData{0, 0.0} {}

    HILBaroData(uint64_t t, float p) : PressureData{t, p} {}

    static std::string header() { return "timestamp,press\n"; }

    void print(std::ostream& os) const
    {
        os << pressureTimestamp << "," << pressure << "\n";
    }
};
