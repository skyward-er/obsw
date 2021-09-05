/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <LoggerService/LoggerService.h>
#include <TelemetriesTelecommands/Mavlink.h>
#include <diagnostic/PrintLogger.h>

namespace DeathStackBoard
{

void handleMavlinkMessage(MavDriver* mav_driver, const mavlink_message_t& msg);

void sendAck(MavDriver* mav_driver, const mavlink_message_t& msg);

bool sendTelemetry(MavDriver* mav_driver, const uint8_t tm_id);

void logMavlinkStatus(MavDriver* mav_driver);

static LoggerService* logger = LoggerService::getInstance();

static PrintLogger print_logger =
    Logging::getLogger("deathstack.tmtc.tchandler");

}  // namespace DeathStackBoard