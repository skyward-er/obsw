/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#pragma once

#include <stdint.h>

#include <iostream>
#include <string>

namespace RIG
{
// This class defines all the types received from the CAN bus
struct CanPressureSensor
{
    uint64_t timestamp = 0;
    uint8_t sensorId   = 0;
    float pressure     = 0;

    static std::string header() { return "timestamp,sensorId,pressure"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)sensorId << "," << pressure << "\n";
    }
};

struct CanTemperatureSensor
{
    uint64_t timestamp = 0;
    uint8_t sensorId   = 0;
    float temperature  = 0;

    static std::string header() { return "timestamp,sensorId,temperature"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)sensorId << "," << temperature << "\n";
    }
};

struct CanCurrentSensor
{
    uint64_t timestamp = 0;
    uint8_t sensorId   = 0;
    uint8_t boardId    = 0;
    float current      = 0;

    static std::string header() { return "timestamp,sensorId,boardId,current"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)sensorId << "," << (int)boardId << ","
           << current << "\n";
    }
};

struct CanVoltageSensor
{
    uint64_t timestamp = 0;
    uint8_t sensorId   = 0;
    uint8_t boardId    = 0;
    float voltage      = 0;

    static std::string header() { return "timestamp,sensorId,boardId,voltage"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)sensorId << "," << (int)boardId << ","
           << voltage << "\n";
    }
};
}  // namespace RIG