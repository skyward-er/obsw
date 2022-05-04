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

#include <Payload/Main/Radio.h>
#include <Payload/Main/Sensors.h>
#include <Payload/PayloadStatus.h>
#include <events/EventBroker.h>
#include <scheduler/TaskScheduler.h>

namespace Payload
{

/**
 * @brief This class represents the main singleton that keeps all the project
 * objects. It has all the instances and its duty is to initialize them.
 *
 */
class Payload : public Boardcore::Singleton<Payload>
{
    friend class Boardcore::Singleton<Payload>;

public:
    // Event broker used all around the code for the event pattern
    Boardcore::EventBroker* broker;

    // Task scheduler to schedule some regular functions over time
    Boardcore::TaskScheduler* scheduler;

    // Collection of all the sensors. It samples it automatically through the
    // task scheduler
    Sensors* sensors;

    // Radio class that manages all the comunication stuff and handles all the
    // ground station requests
    Radio* radio;

    /**
     * @brief Method to start all the macro obsw elements
     */
    void start() {}

    /**
     * @brief Method to start the SD logging and log the SD status
     */
    void startSDlogger() {}

private:
    // Printlogger for debug and errors
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Payload");

    // SD logger accessible to everyone
    Boardcore::Logger* SDlogger;

    // Struct that resumes if at macro level there were some problems
    PayloadStatus status{};

    Payload() {}
};

}  // namespace Payload