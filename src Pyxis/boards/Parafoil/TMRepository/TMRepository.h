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

#include <Singleton.h>
#include <common/Mavlink.h>
#include <diagnostic/PrintLogger.h>

/**
 * @brief This class represents the collection of data that can be sent via
 * radio communication. This refers to mavlink libraries and structures created
 * in the correct .xml file.
 *
 * It is necessary that this singleton class handles the structure update
 * (when a message pack is requested).
 * The pack method is the core of the class. It returns a mavlink_message
 * with the message data(specified with the id) requested.
 */

namespace Parafoil
{

class TMRepository : public Boardcore::Singleton<TMRepository>
{
    friend class Boardcore::Singleton<TMRepository>;

public:
    /**
     * @brief Retrieve a system telemetry message in packed form.
     *
     * @param reqTm Required telemetry.
     * @return Packed mavlink telemetry or a nack.
     */
    mavlink_message_t packSystemTm(SystemTMList reqTm);

    /**
     * @brief Retrieve a sensor telemetry message in packed form.
     *
     * @param reqTm Required telemetry.
     * @return Packed mavlink telemetry or a nack.
     */
    mavlink_message_t packSensorsTm(SensorsTMList reqTm);

private:
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("tmrepo");
};

}  // namespace Parafoil
