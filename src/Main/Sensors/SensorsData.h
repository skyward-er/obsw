/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Pietro Bortolus
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

#pragma once

#include <drivers/canbus/CanProtocol/CanProtocolTypes.h>
#include <sensors/AS5047D/AS5047DData.h>
#include <sensors/LIS2MDL/LIS2MDLData.h>
#include <sensors/LSM6DSRX/LSM6DSRXData.h>
#include <sensors/SensorData.h>

#include <reflect.hpp>

namespace Main
{

struct StaticPressure0Data : public Boardcore::PressureData
{
    explicit StaticPressure0Data(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    StaticPressure0Data() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(StaticPressure0Data,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct StaticPressure1Data : Boardcore::PressureData
{
    explicit StaticPressure1Data(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    StaticPressure1Data() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(StaticPressure1Data,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct StaticPressure2Data : Boardcore::PressureData
{
    explicit StaticPressure2Data(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    StaticPressure2Data() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(StaticPressure2Data,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct LSM6DSRX0Data : Boardcore::LSM6DSRXData
{
    explicit LSM6DSRX0Data(const Boardcore::LSM6DSRXData& data)
        : Boardcore::LSM6DSRXData(data)
    {
    }

    LSM6DSRX0Data() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(LSM6DSRX0Data, EXTEND_DEF(Boardcore::LSM6DSRXData));
    }
};

struct LSM6DSRX1Data : Boardcore::LSM6DSRXData
{
    explicit LSM6DSRX1Data(const Boardcore::LSM6DSRXData& data)
        : Boardcore::LSM6DSRXData(data)
    {
    }

    LSM6DSRX1Data() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(LSM6DSRX1Data, EXTEND_DEF(Boardcore::LSM6DSRXData));
    }
};

struct LIS2MDLExternalData : Boardcore::LIS2MDLData
{
    explicit LIS2MDLExternalData(const Boardcore::LIS2MDLData& data)
        : Boardcore::LIS2MDLData(data)
    {
    }

    LIS2MDLExternalData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(LIS2MDLExternalData,
                          EXTEND_DEF(Boardcore::LIS2MDLData));
    }
};

struct AS5047DLeftData : Boardcore::AS5047DData
{
    explicit AS5047DLeftData(const Boardcore::AS5047DData& data)
        : Boardcore::AS5047DData(data)
    {
    }

    AS5047DLeftData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(AS5047DLeftData, EXTEND_DEF(Boardcore::AS5047DData));
    }
};

struct AS5047DRightData : Boardcore::AS5047DData
{
    explicit AS5047DRightData(const Boardcore::AS5047DData& data)
        : Boardcore::AS5047DData(data)
    {
    }

    AS5047DRightData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(AS5047DRightData, EXTEND_DEF(Boardcore::AS5047DData));
    }
};

struct PitotStaticPressureData : Boardcore::PressureData
{
    explicit PitotStaticPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    PitotStaticPressureData() {}

    PitotStaticPressureData(Boardcore::CanPressureData data)
    {
        pressureTimestamp = data.pressureTimestamp;
        pressure          = data.pressure;
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(PitotStaticPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct PitotTotalPressureData : Boardcore::PressureData
{
    explicit PitotTotalPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    PitotTotalPressureData() {}

    PitotTotalPressureData(Boardcore::CanPressureData data)
    {
        pressureTimestamp = data.pressureTimestamp;
        pressure          = data.pressure;
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(PitotTotalPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct CalibrationData
{
    uint64_t timestamp     = 0;
    float accLowBiasX      = 0.0f;
    float accLowBiasY      = 0.0f;
    float accLowBiasZ      = 0.0f;
    float gyroLowBiasX     = 0.0f;
    float gyroLowBiasY     = 0.0f;
    float gyroLowBiasZ     = 0.0f;
    float accHighBiasX     = 0.0f;
    float accHighBiasY     = 0.0f;
    float accHighBiasZ     = 0.0f;
    float gyroHighBiasX    = 0.0f;
    float gyroHighBiasY    = 0.0f;
    float gyroHighBiasZ    = 0.0f;
    float vn100AccBiasX    = 0.0f;
    float vn100AccBiasY    = 0.0f;
    float vn100AccBiasZ    = 0.0f;
    float vn100GyroBiasX   = 0.0f;
    float vn100GyroBiasY   = 0.0f;
    float vn100GyroBiasZ   = 0.0f;
    float magBiasX         = 0.0f;
    float magBiasY         = 0.0f;
    float magBiasZ         = 0.0f;
    float magScaleX        = 0.0f;
    float magScaleY        = 0.0f;
    float magScaleZ        = 0.0f;
    float pitotDynamicBias = 0.0f;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            CalibrationData,
            FIELD_DEF(timestamp) FIELD_DEF(accLowBiasX) FIELD_DEF(
                accLowBiasY) FIELD_DEF(accLowBiasZ) FIELD_DEF(gyroLowBiasX)
                FIELD_DEF(gyroLowBiasY) FIELD_DEF(gyroLowBiasZ) FIELD_DEF(
                    accHighBiasX) FIELD_DEF(accHighBiasY)
                    FIELD_DEF(accHighBiasZ) FIELD_DEF(gyroHighBiasX) FIELD_DEF(
                        gyroHighBiasY) FIELD_DEF(gyroHighBiasZ)
                        FIELD_DEF(vn100AccBiasX) FIELD_DEF(vn100AccBiasY)
                            FIELD_DEF(vn100AccBiasZ) FIELD_DEF(vn100GyroBiasX)
                                FIELD_DEF(vn100GyroBiasY)
                                    FIELD_DEF(vn100GyroBiasZ)
                                        FIELD_DEF(magBiasX) FIELD_DEF(magBiasY)
                                            FIELD_DEF(magBiasZ)
                                                FIELD_DEF(magScaleX)
                                                    FIELD_DEF(magScaleY)
                                                        FIELD_DEF(magScaleZ));
    }
};

}  // namespace Main
