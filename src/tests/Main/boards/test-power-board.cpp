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

namespace testBuzzer
{
#include "../actuators/test-buzzer.cpp"
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
                testBuzzer::main();
                break;
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
        " 2. for buzzer test, enjoy!\n");
    iprintf("\n>> ");
    getline(cin, temp);
    stringstream(temp) >> choice;

    return choice;
}
