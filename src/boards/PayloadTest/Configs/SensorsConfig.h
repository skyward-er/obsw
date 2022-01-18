/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

/**
 * This class specifies the sensors constants that the sensor manager
 * needs to know about every device. For example the sample time is
 * essential to understand how much time a sensor should wait before
 * another sample.
 */

#pragma once

#include <sensors/LIS3DSH/LIS3DSH.h>

namespace ParafoilTest
{
    static constexpr unsigned int SAMPLE_PERIOD_ACCEL_LIS3DSH = 2; //2 millis

    static constexpr LIS3DSH::OutputDataRate    ACCEL_LIS3DSH_ODR           = LIS3DSH::ODR_800_HZ;
    static constexpr LIS3DSH::BlockDataUpdate   ACCEL_LIS3DSH_BDU           = LIS3DSH::UPDATE_AFTER_READ_MODE;
    static constexpr LIS3DSH::FullScale         ACCEL_LIS3DSH_FULL_SCALE    = LIS3DSH::FULL_SCALE_2G;
}