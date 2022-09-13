/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#include <ActiveObject.h>
#include <Singleton.h>
#include <drivers/usart/USART.h>

namespace Ciuti
{

class SerialWatcher : public Boardcore::ActiveObject
{
public:
    SerialWatcher(Boardcore::USART &usart) : usart(usart) {}

private:
    void run() override;

    Boardcore::USART &usart;
};

class SerialWatcherController
    : public Boardcore::Singleton<SerialWatcherController>
{
    friend class Boardcore::Singleton<SerialWatcherController>;

public:
    void start();
    void stop();

private:
    SerialWatcherController();
    ~SerialWatcherController();

    SerialWatcher *serial_watcher1 = nullptr;
    SerialWatcher *serial_watcher2 = nullptr;
};

}  // namespace Ciuti