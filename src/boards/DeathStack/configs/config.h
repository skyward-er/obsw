/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

static constexpr unsigned int DEFERRED_EVENTS_QUEUE_SIZE = 100;

// Default reference values settings
#ifdef EUROC
static const float DEFAULT_REFERENCE_ALTITUDE = 160.0f;
static const float DEFAULT_REFERENCE_PRESSURE = 100022.4f;
#else
static const float DEFAULT_REFERENCE_ALTITUDE     = 1420.0f;
static const float DEFAULT_REFERENCE_PRESSURE     = 85389.4f;
#endif

static const float DEFAULT_REFERENCE_TEMPERATURE = 288.15f;

// Deployment altitude AGL
static const float DEFAULT_DEPLOYMENT_ALTITUDE = 350;
