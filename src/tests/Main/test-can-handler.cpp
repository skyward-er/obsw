/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <Main/CanHandler/CanHandler.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace Main;

constexpr int PERIOD = 5 * 1000;  // [s]

int main()
{
    CanHandler::getInstance().start();

    while (true)
    {
        CanHandler::getInstance().sendArmCommand();
        printf("sendArmEvent\n");
        Thread::sleep(PERIOD);

        CanHandler::getInstance().sendDisarmCommand();
        printf("sendDisarmCommand\n");
        Thread::sleep(PERIOD);

        CanHandler::getInstance().sendCamOnCommand();
        printf("sendCamOnCommand\n");
        Thread::sleep(PERIOD);

        CanHandler::getInstance().sendCamOffCommand();
        printf("sendCamOffCommand\n");
        Thread::sleep(PERIOD);

        CanHandler::getInstance().sendLiftoffEvent();
        printf("sendLiftoffEvent\n");
        Thread::sleep(PERIOD);

        CanHandler::getInstance().sendApogeeEvent();
        printf("sendApogeeEvent\n");
        Thread::sleep(PERIOD);

        CanHandler::getInstance().sendLandingEvent();
        printf("sendLandingEvent\n");
        Thread::sleep(PERIOD);
    }
}
