/*
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Vincenzo Santomarco
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

#include <math.h>

namespace DeathStackBoard
{

class TrajectoryPoint
{
public:
    TrajectoryPoint() : TrajectoryPoint(0, 0) {}
    TrajectoryPoint(float z, float vz) : z(z), vz(vz) {}

    float getZ() { return z; }
    float getVz() { return vz; }

    static float distance(TrajectoryPoint a, TrajectoryPoint b)
    {
        return powf(a.getZ() - b.getZ(), 2) + powf(a.getVz() - b.getVz(), 2);
    }

    static float zDistance(TrajectoryPoint a, TrajectoryPoint b)
    {
        return abs(a.getZ() - b.getZ());
    }

    static float vzDistance(TrajectoryPoint a, TrajectoryPoint b)
    {
        return abs(a.getVz() - b.getVz());
    }

private:
    float z;
    float vz;
};

}  // namespace DeathStackBoard