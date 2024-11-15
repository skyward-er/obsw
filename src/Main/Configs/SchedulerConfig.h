/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <miosix.h>

namespace Main
{

namespace Config
{

namespace Scheduler
{

// Used for NAS related activities (state machines/scheduler)
static const miosix::Priority NAS_PRIORITY = miosix::PRIORITY_MAX - 1;
// Used for MEA related activities (state machines/scheduler)
static const miosix::Priority MEA_PRIORITY = miosix::PRIORITY_MAX - 1;
// Used for ABK related activities (state machines/scheduler)
static const miosix::Priority ABK_PRIORITY = miosix::PRIORITY_MAX - 1;
// Used for ADA related activities (state machines/scheduler)
static const miosix::Priority ADA_PRIORITY = miosix::PRIORITY_MAX - 1;
// Used for Sensors TaskScheduler
static const miosix::Priority SENSORS_PRIORITY = miosix::PRIORITY_MAX - 2;
// Used for everything else:
// - Radio periodic telemetry
// - CanBus periodic heartbeat
// - Actuators buzzer
static const miosix::Priority OTHERS_PRIORITY = miosix::PRIORITY_MAX - 3;

// Used for FlightModeManager
static const miosix::Priority FMM_PRIORITY = miosix::PRIORITY_MAX - 1;

}  // namespace Scheduler

}  // namespace Config

}  // namespace Main