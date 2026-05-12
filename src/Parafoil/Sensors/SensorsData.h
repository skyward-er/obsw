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

#include <sensors/AS5047D/AS5047DData.h>
#include <sensors/LIS2MDL/LIS2MDLData.h>
#include <sensors/LSM6DSRX/LSM6DSRXData.h>
#include <sensors/SensorData.h>

#include <reflect.hpp>

namespace Parafoil
{

struct StaticPressureData : public Boardcore::PressureData
{
    explicit StaticPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    StaticPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(StaticPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
    }
};

struct DplBayPressureData : Boardcore::PressureData
{
    explicit DplBayPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    DplBayPressureData() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(DplBayPressureData,
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

struct DynamicPressureData : public Boardcore::PressureData
{
    DynamicPressureData() = default;

    explicit DynamicPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(DynamicPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
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

struct AS5047D0Data : Boardcore::AS5047DData
{
    explicit AS5047D0Data(const Boardcore::AS5047DData& data)
        : Boardcore::AS5047DData(data)
    {
    }

    AS5047D0Data() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(AS5047D0Data, EXTEND_DEF(Boardcore::AS5047DData));
    }
};

struct AS5047D1Data : Boardcore::AS5047DData
{
    explicit AS5047D1Data(const Boardcore::AS5047DData& data)
        : Boardcore::AS5047DData(data)
    {
    }

    AS5047D1Data() {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(AS5047D1Data, EXTEND_DEF(Boardcore::AS5047DData));
    }
};

struct CalibrationData
{
    uint64_t timestamp     = 0;
    float acc0BiasX        = 0.0f;
    float acc0BiasY        = 0.0f;
    float acc0BiasZ        = 0.0f;
    float gyro0BiasX       = 0.0f;
    float gyro0BiasY       = 0.0f;
    float gyro0BiasZ       = 0.0f;
    float acc1BiasX        = 0.0f;
    float acc1BiasY        = 0.0f;
    float acc1BiasZ        = 0.0f;
    float gyro1BiasX       = 0.0f;
    float gyro1BiasY       = 0.0f;
    float gyro1BiasZ       = 0.0f;
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
            FIELD_DEF(timestamp) FIELD_DEF(acc0BiasX) FIELD_DEF(acc0BiasY)
                FIELD_DEF(acc0BiasZ) FIELD_DEF(gyro0BiasX) FIELD_DEF(gyro0BiasY)
                    FIELD_DEF(gyro0BiasZ) FIELD_DEF(acc1BiasX) FIELD_DEF(
                        acc1BiasY) FIELD_DEF(acc1BiasZ) FIELD_DEF(gyro1BiasX)
                        FIELD_DEF(gyro1BiasY) FIELD_DEF(gyro1BiasZ)
                            FIELD_DEF(magBiasX) FIELD_DEF(magBiasY)
                                FIELD_DEF(magBiasZ) FIELD_DEF(magScaleX)
                                    FIELD_DEF(magScaleY) FIELD_DEF(magScaleZ));
    }
};

}  // namespace Parafoil
