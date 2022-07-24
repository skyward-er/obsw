/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Federico Mandelli, Alberto Nidasio
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

#include <drivers/canbus/CanProtocol/CanProtocol.h>

namespace Main
{

class CanHandler : public Boardcore::Singleton<CanHandler>
{
    friend Boardcore::Singleton<CanHandler>;

public:
    /**
     * @brief Starts the CanProtocol.
     */
    bool start();

    /**
     * @brief Tells whether the can protocol was started.
     */
    bool isStarted();

    // Boardcore::Canbus::CanRXStatus getCanStatus();

    // void logStatus();

private:
    CanHandler();

    void handleCanMessage(const Boardcore::Canbus::CanMessage &msg);

    Boardcore::Canbus::CanbusDriver *driver;
    Boardcore::Canbus::CanProtocol *protocol;
};

}  // namespace Main
