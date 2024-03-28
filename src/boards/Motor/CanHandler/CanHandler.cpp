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

#include "CanHandler.h"

#include <common/CanConfig.h>

using namespace Motor;
using namespace Boardcore;
using namespace Boardcore::Canbus;
using namespace Common;

CanHandler::CanHandler()
{
    CanbusDriver::AutoBitTiming bitTiming;
    bitTiming.baudRate    = CanConfig::BAUD_RATE;
    bitTiming.samplePoint = CanConfig::SAMPLE_POINT;

    CanbusDriver::CanbusConfig config;

    driver = std::make_unique<CanbusDriver>(CAN1, config, bitTiming);

    protocol = std::make_unique<CanProtocol>(
        driver.get(),
        [this](const Canbus::CanMessage &msg) { handleCanMessage(msg); }, 3);

    protocol->addFilter(static_cast<uint8_t>(CanConfig::Board::RIG),
                        static_cast<uint8_t>(CanConfig::Board::BROADCAST));
    protocol->addFilter(static_cast<uint8_t>(CanConfig::Board::MAIN),
                        static_cast<uint8_t>(CanConfig::Board::BROADCAST));

    driver->init();
}

bool CanHandler::start() { return protocol->start(); }

void CanHandler::sendEvent(Common::CanConfig::EventId event)
{
    protocol->enqueueEvent(static_cast<uint8_t>(CanConfig::Priority::CRITICAL),
                           static_cast<uint8_t>(CanConfig::PrimaryType::EVENTS),
                           static_cast<uint8_t>(CanConfig::Board::MAIN),
                           static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                           static_cast<uint8_t>(event));
}

void CanHandler::handleCanMessage(const Canbus::CanMessage &msg)
{
    LOG_INFO(logger, "Received can message {}", msg.getPrimaryType(),
             msg.getSecondaryType());
}
