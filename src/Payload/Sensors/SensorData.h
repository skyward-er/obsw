
/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <sensors/SensorData.h>

#include <reflect.hpp>

namespace Payload
{

// Wrappers to differentiate static and dynamic pressure for logging

struct StaticPressureData : public Boardcore::PressureData
{
    StaticPressureData() = default;

    explicit StaticPressureData(const Boardcore::PressureData& data)
        : Boardcore::PressureData(data)
    {
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(StaticPressureData,
                          EXTEND_DEF(Boardcore::PressureData));
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

struct SensorCalibrationData
{
    uint64_t timestamp      = 0;
    float gyroBiasX         = 0.0f;
    float gyroBiasY         = 0.0f;
    float gyroBiasZ         = 0.0f;
    float magBiasX          = 0.0f;
    float magBiasY          = 0.0f;
    float magBiasZ          = 0.0f;
    float magScaleX         = 0.0f;
    float magScaleY         = 0.0f;
    float magScaleZ         = 0.0f;
    float staticPressBias   = 0.0f;
    float staticPressScale  = 0.0f;
    float dynamicPressBias  = 0.0f;
    float dynamicPressScale = 0.0f;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            SensorCalibrationData,
            FIELD_DEF(timestamp) FIELD_DEF(gyroBiasX) FIELD_DEF(gyroBiasY)
                FIELD_DEF(gyroBiasZ) FIELD_DEF(magBiasX) FIELD_DEF(magBiasY)
                    FIELD_DEF(magBiasZ) FIELD_DEF(magScaleX)
                        FIELD_DEF(magScaleY) FIELD_DEF(magScaleZ)
                            FIELD_DEF(staticPressBias)
                                FIELD_DEF(staticPressScale)
                                    FIELD_DEF(dynamicPressBias)
                                        FIELD_DEF(dynamicPressScale));
    }
};

}  // namespace Payload
