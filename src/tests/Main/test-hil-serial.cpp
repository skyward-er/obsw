/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <miosix.h>

#include "drivers/usart/USART.h"

using namespace miosix;

void main()
{
    size_t len = 32;
    char buf[len];
    Boardcore::USART usart3(USART3,
                            Boardcore::USARTInterface::Baudrate::B115200);
    usart3.init();

    for (int i = 0; i < 10; i++)
    {
        iprintf("should send message out of HIL port\n");
        usart3.writeString("This is a message out of the HIL port!\r\n");
        Thread::sleep(200);
    }

    iprintf("Testing receiving data: input a string from HIL serial port\n");
    usart3.writeString("Input something: ");
    usart3.read(buf, len);
    printf("received: '%s'\n", buf);

    iprintf("Input another string\n");
    usart3.writeString("Input something again: ");
    usart3.read(buf, len);
    printf("received: '%s'\n", buf);

    iprintf("test HIL serial port over!\n\n");
}