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

namespace testBMX160
{
#include "../sensors/test-bmx160.cpp"
}

namespace testMPU9250
{
#include "../sensors/test-mpu9250.cpp"
}

namespace testMS5803
{
#include "../sensors/test-ms5803.cpp"
}

namespace testADS131
{
#include "../sensors/test-ads131.cpp"
}

int menu();

void testSerialDebug()
{
    iprintf(
        "If you are seeing the menu why whould you test this?! you f***g "
        "donkey!\n\n");
}

void testGPIOInput(::miosix::GpioPin pin);

int main()
{
    while (true)
    {
        switch (menu())
        {
            case 1:
                testSerialDebug();
                break;
            case 2:
                testBMX160::main();
                break;
            case 3:
                testMPU9250::main();
                break;
            case 4:
                testMS5803::main();
                break;
            case 5:
                testADS131::main();
                break;
            case 99:
                return 0;
        }
    }

    return 0;
}

int menu()
{
    string temp;
    int choice;

    iprintf(
        "Type:\n"
        " 1. for debug-serial test\n"
        " 2. for IMU bmx160 test\n"
        " 3. for IMU mpu9250 test\n"
        " 4. for digital pressure sensor ms5803 test\n"
        " 5. for analogic pressure sensors with ads131 test\n"
        " 99. go back\n");
    iprintf("\n>> ");
    getline(cin, temp);
    stringstream(temp) >> choice;

    return choice;
}
