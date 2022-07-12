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

#pragma once

#include <Payload/Control/NASController.h>
#include <algorithms/NAS/NAS.h>
#include <diagnostic/PrintLogger.h>
#include <logger/Logger.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/BMX160/BMX160Data.h>
#include <sensors/UBXGPS/UBXGPSData.h>

namespace Payload
{

class Algorithms
{
public:
    // All the algorithms that need to be started
    NASController<Boardcore::BMX160Data, Boardcore::UBXGPSData>* nas;

    // The scheduler
    Boardcore::TaskScheduler* scheduler;

    /**
     * @brief Construct a new Algorithms object
     *
     * @param scheduler The algorithms task scheduler
     */
    Algorithms(Boardcore::TaskScheduler* scheduler);

    /**
     * @brief Destroy the Algorithms object
     */
    ~Algorithms();

    /**
     * @brief Method to start the algorithms task scheduler
     */
    bool start();

    // Kernel locked getters
    Boardcore::NASState getNASLastSample();

private:
    // Debug serial logger
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("algorithms");

    // SD logger
    Boardcore::Logger* SDlogger = &Boardcore::Logger::getInstance();

    // Init functions
    void NASInit();
};

}  // namespace Payload