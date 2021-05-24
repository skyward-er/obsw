/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano
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

/**
 * @brief Configuration file that includes only the right structures described
 * in the config file of the test.
 *
 * Usage:
 * #elif <Flag>
 * #include "<test-directory>/HILSimulationConfig.h"
 *
 * REMEMBER:
 * when defining the entry in "sbs.conf" you should add to the defines -D<Flag>
 *
 * WARNING:
 * You should always CLEAN your board before flashing a new entrypoint. Some
 * flags could still be in memory
 */

/* Hardware in the loop entrypoint */
#if defined(HARDWARE_IN_THE_LOOP)
#include "entrypoints/hardware_in_the_loop/HILSimulationConfig.h"
/* serial comunication test */
#elif defined(HIL_SERIALINTERFACE)
#include "hardware_in_the_loop/test-SerialInterface/HILSimulationConfig.h"
/* serial simulation with sample manager */
#elif defined(HIL)
#include "hardware_in_the_loop/test-HIL/HILSimulationConfig.h"
#elif defined(HIL_ADA)
#include "hardware_in_the_loop/test-HIL+ADA/HILSimulationConfig.h"
#elif defined(HIL_AEROBRAKE)
#include "hardware_in_the_loop/test-HIL+Aerobrake/HILSimulationConfig.h"
#elif defined(HIL_ADA_AEROBRAKE)
#include "hardware_in_the_loop/test-HIL+ADA+Aerobrake/HILSimulationConfig.h"
#elif defined(HIL_ADA_AEROBRAKECONTROLLER)
#include "hardware_in_the_loop/test-HIL+ADA+AerobrakeController/HILSimulationConfig.h"
#elif defined(HIL_ADA_AEROBRAKECONTROLLER_NAS)
#include "hardware_in_the_loop/test-HIL+ADA+AerobrakeController+nas/HILSimulationConfig.h"
/*
#elif defined(HIL_<tuoFlag>)
#include "<test-directory>/HILSimulationConfig.h"
*/
#else
#error You have add the flag of your configuration file for the HIL testing!
#endif