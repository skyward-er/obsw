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

#include <Ciuti/BoardScheduler.h>
#include <Ciuti/Buses.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <utils/Debug.h>

using namespace miosix;
using namespace Boardcore;

// Simple xorshift RNG
uint32_t xorshift32()
{
    // https://xkcd.com/221/
    static uint32_t STATE = 0x08104444;

    uint32_t x = STATE;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;

    // Do I really need to lock this? Nah, introduce extra randomness.
    return STATE = x;
}

namespace Ciuti
{

SerialWatcher::SerialWatcher(USART &usart, unsigned int id) : usart(usart)
{
    stats.usart_id = id;
}

void SerialWatcher::run()
{
    while (isRunning())
    {
        long long now = getTick();

        uint32_t sent = xorshift32();
        uint32_t recv;

        // Perform a loopback read
        usart.clearQueue();
        usart.write(&sent, 4);
        usart.read(&recv, 4);

        if (sent != recv)
            stats.error_count += 1;

        stats.last_timestamp = TimestampTimer::getTimestamp();
        Thread::sleepUntil(now + SerialWatcher::PERIOD);
    }
}

void SerialWatcherController::start()
{
    serial_watcher1->start();
    serial_watcher2->start();

    BoardScheduler::getInstance().getScheduler().addTask(
        [=]() { Logger::getInstance().log(this->serial_watcher1->getStats()); },
        SerialWatcher::PERIOD);

    BoardScheduler::getInstance().getScheduler().addTask(
        [=]() { Logger::getInstance().log(this->serial_watcher2->getStats()); },
        SerialWatcher::PERIOD);

    LOG_INFO(logger, "Serial watcher controller setup done!");
}

SerialWatcherController::SerialWatcherController()
{
    serial_watcher1 = new SerialWatcher(Ciuti::Buses::getInstance().usart2, 2);
    serial_watcher2 = new SerialWatcher(Ciuti::Buses::getInstance().usart3, 3);
}

SerialWatcherController::~SerialWatcherController()
{
    delete serial_watcher1;
    delete serial_watcher2;
}

}  // namespace Ciuti