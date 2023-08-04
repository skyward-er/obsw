/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include <common/Mavlink.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Gs
{

/**
 * @brief Central hub connecting all outgoing and ingoing modules.
 */
class Hub : public Boardcore::Module
{
public:
    Hub() {}

    /**
     * @brief Dispatch to the correct interface and outgoing packet (gs ->
     * rocket).
     */
    void dispatchOutgoingMsg(const mavlink_message_t& msg);

    /**
     * @brief Dispatch to the correct interface and incoming packet (rocket ->
     * gs).
     */
    void dispatchIncomingMsg(const mavlink_message_t& msg);

private:
    /**
     * @brief Used internally to signal to the gs computer that something went
     * wrong, and the packet could not be delivered.
     */
    void sendNack(const mavlink_message_t& msg);
};

}  // namespace Gs