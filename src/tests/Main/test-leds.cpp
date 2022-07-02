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

using namespace miosix;

using led1 = miosix::leds::green1;
using led2 = miosix::leds::red;
using led3 = miosix::leds::blue;
using led4 = miosix::leds::green2;

int main()
{
    led4::low();
    led1::low();
    led2::low();
    led3::low();

    for (int i = 0; i < 10; i++)
    {
        printf("should blink\n");
        led4::low();
        led1::high();
        Thread::sleep(200);

        led1::low();
        led2::high();
        Thread::sleep(200);

        led2::low();
        led3::high();
        Thread::sleep(200);

        led3::low();
        led4::high();
        Thread::sleep(200);
    }

    led1::low();
    led2::low();
    led3::low();
    led4::low();

    printf("test leds over!\n\n");

    return 0;
}
