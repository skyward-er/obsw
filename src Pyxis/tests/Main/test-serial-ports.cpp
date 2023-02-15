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

#include <drivers/usart/USART.h>
#include <miosix.h>

#include <thread>

using namespace miosix;
using namespace Boardcore;

int main()
{
    USART debug(USART1, USARTInterface::Baudrate::B115200);
    USART hil(USART3, USARTInterface::Baudrate::B115200);

    debug.init();
    hil.init();

    std::thread debugPing(
        [&]()
        {
            char byte = 'q';
            while (true)
            {
                debug.read(&byte, 1);
                debug.writeString("You typed something\n\r");
            }
        });
    std::thread hilPing(
        [&]()
        {
            char byte;
            while (true)
            {
                hil.read(&byte, 1);
                hil.writeString("You typed something\n\r");
            }
        });

    while (true)
    {
        debug.writeString("This is the debug port!\n\r");
        hil.writeString("This is the hil port!\n\r");

        Thread::sleep(500);
    }
}
