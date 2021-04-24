/*
 * Copyright (c) 2019-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio
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

#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include "utils/PinObserver.h"

namespace DeathStackBoard
{

static const unsigned int PIN_POLL_INTERVAL = 10;  // ms

// Launch pin config
static const GpioPin launch_pin(miosix::inputs::lp_dtch::getPin());
static const PinObserver::Transition TRIGGER_LAUNCH_PIN =
    PinObserver::Transition::FALLING_EDGE;
// How many consecutive times the launch pin should be detected as detached
// before triggering a launch event.
static const unsigned int THRESHOLD_LAUNCH_PIN = 10;

// Nosecone detach pin config
static const GpioPin nosecone_pin(miosix::inputs::nc_dtch::getPin());
static const PinObserver::Transition TRIGGER_NC_DETACH_PIN =
    PinObserver::Transition::FALLING_EDGE;
// How many consecutive times the nosecone pin should be detected as detached
// before triggering a nosecone detach event.
static const unsigned int THRESHOLD_NC_DETACH_PIN = 10;

// Dpl servo actuation pin config
static const GpioPin dpl_servo_pin(miosix::inputs::expulsion_in::getPin());
static const PinObserver::Transition TRIGGER_DPL_SERVO_PIN =
    PinObserver::Transition::FALLING_EDGE;
// How many consecutive times the deployment servo pin should be detected as
// detached before triggering a nosecone detach event.
static const unsigned int THRESHOLD_DPL_SERVO_PIN = 10;

}  // namespace DeathStackBoard