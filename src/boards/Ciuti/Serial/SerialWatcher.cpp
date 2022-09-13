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

#include "SerialWatcher.h"

#include <utils/Debug.h>
#include <Ciuti/Buses.h>
#include <miosix.h>

namespace Ciuti
{

void SerialWatcher::run()
{
    while(isRunning()) {
        uint8_t sent = 25;
        uint8_t recv;

        usart.clearQueue();
        usart.write(&sent, 1);

        usart.read(&recv, 1);

        miosix::Thread::sleep(1000);
    }
}

void SerialWatcherController::start()
{
    serial_watcher1->start();
    serial_watcher2->start();
}

void SerialWatcherController::stop()
{
    serial_watcher1->stop();
    serial_watcher2->stop();
}

SerialWatcherController::SerialWatcherController() 
{
    serial_watcher1 = new SerialWatcher(Ciuti::Buses::getInstance().usart2);
    serial_watcher2 = new SerialWatcher(Ciuti::Buses::getInstance().usart3);
}

SerialWatcherController::~SerialWatcherController() 
{
    delete serial_watcher1;
    delete serial_watcher2;
}

}