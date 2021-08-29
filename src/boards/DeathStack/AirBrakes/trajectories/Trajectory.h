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
 * THE SOFTWARE
 */

#pragma once

#include "Trajectories_data.h"
#include "TrajectoryPoint.h"

namespace DeathStackBoard
{

class Trajectory
{
private:
    uint8_t index;  // trajectory id, from 0 to 9, generated with openings from
                    // 10% to 90%
    float s_bar;    // fixed area for trajectory generation
                    // if error = 0, this is the needed airbrakes area

public:
    Trajectory(uint8_t index, float s_max)
        : index(index),
          // (index + 1) / 10 : airbrakes opening percentage, from 0.1 to 0.9
          s_bar(((index + 1) / 10.0f) * s_max)
    {
    }

    Trajectory() : index(0), s_bar(0.0f) {}

    Trajectory& operator=(const Trajectory& other)
    {
        this->index = other.index;
        this->s_bar = other.s_bar;
        return *this;
    }

    uint32_t length() { return TRAJECTORIES_DATA[index].length; }

    TrajectoryPoint get(uint32_t idx)
    {
        point_t point = TRAJECTORIES_DATA[index].data[idx];
        return TrajectoryPoint(point.z, point.vz);
    }

    uint8_t getTrajectoryIndex() { return index; }

    float getRefSurface() { return s_bar; }
};

}  // namespace DeathStackBoard
