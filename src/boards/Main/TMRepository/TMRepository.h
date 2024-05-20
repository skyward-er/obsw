/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include <common/MavlinkGemini.h>
#include <diagnostic/PrintLogger.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Main
{
class TMRepository : public Boardcore::Module
{
public:
    inline TMRepository() {}

    // Packs the telemetry at system level (E.g. logger telemetry..)
    mavlink_message_t packSystemTm(SystemTMList tmId, uint8_t msgId,
                                   uint8_t seq);

    // Packs the telemetry at sensors level (E.g. pressure sensors..)
    mavlink_message_t packSensorsTm(SensorsTMList sensorId, uint8_t msgId,
                                    uint8_t seq);

    // Packs the telemetry about servo positions states
    mavlink_message_t packServoTm(ServosList servoId, uint8_t msgId,
                                  uint8_t seq);

private:
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("TMRepository");
};
}  // namespace Main