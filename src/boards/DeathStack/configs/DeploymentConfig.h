/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#ifndef SRC_SHARED_BOARDS_HOMEONE_DEPLOYMENTCONTROLLER_DEPLOYMENTCONFIG_H
#define SRC_SHARED_BOARDS_HOMEONE_DEPLOYMENTCONTROLLER_DEPLOYMENTCONFIG_H



namespace DeathStackBoard
{

// TODO: Update with correct values

static constexpr unsigned int DEFERRED_EVENTS_BUFFER_SIZE = 10;

static constexpr int MAXIMUM_CUTTING_DURATION = 1000;

static constexpr int NC_MINIMUM_OPENING_TIME = 1000;
static constexpr int NC_OPEN_TIMEOUT         = 1000;
static constexpr int NC_CLOSE_TIMEOUT        = 1000;

static constexpr MotorDirection MOTOR_OPEN_DIR = MotorDirection::NORMAL;
static constexpr MotorDirection MOTOR_CLOSE_DIR = MotorDirection::REVERSE;

static constexpr float MOTOR_OPEN_DUTY_CYCLE = 0.5f;
static constexpr float MOTOR_CLOSE_DUTY_CYCLE = 0.5f;

}  // namespace DeathStackBoard

#endif