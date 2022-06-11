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

namespace testLeds
{
#include "../test-leds.cpp"
}

namespace testSerialHIL
{
#include "../test-hil-serial.cpp"
}

namespace testSDBenchmark
{
#include "../../../skyward-boardcore/src/entrypoints/sdcard-benchmark.cpp"
}

int menu();

void testSerialDebug()
{
    iprintf(
        "If you are seeing the menu why whould you test this?! you f***g "
        "donkey!\n\n");
}

void testGPIOInput(GpioPin pin);

int main()
{
    while (true)
    {
        switch (menu())
        {
            case 1:
                testLeds::main();
                break;
            case 2:
                testSerialDebug();
                break;
            case 3:
                testSerialHIL::main();
                break;
            case 4:
                testGPIOInput(inputs::expulsion::getPin());
                break;
            case 5:
                testGPIOInput(inputs::launchpad::getPin());
                break;
            case 6:
                testGPIOInput(inputs::nosecone_detach::getPin());
                break;
            case 7:
                testGPIOInput(aux::sense_aux_1::getPin());
                break;
            case 8:
                testGPIOInput(aux::sense_aux_2::getPin());
                break;
            case 9:
                testSDBenchmark::main();
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
        " 1. for led wave test\n"
        " 2. for debug-serial test\n"
        " 3. for hil-serial test\n"
        " 4. for expulsion sense test\n"
        " 5. for launchpad sense test\n"
        " 6. for nosecone detachment sense test\n"
        " 7. for auxiliary 1 sense test\n"
        " 8. for auxiliary 2 sense test\n"
        " 9. for SD benchmark test\n");
    iprintf("\n>> ");
    getline(cin, temp);
    stringstream(temp) >> choice;

    return choice;
}

void testGPIOInput(GpioPin pin)
{
    char portName;

    switch (pin.getPort())
    {
        case GPIOA_BASE:
            portName = 'A';
            break;
        case GPIOB_BASE:
            portName = 'B';
            break;
        case GPIOC_BASE:
            portName = 'C';
            break;
        case GPIOD_BASE:
            portName = 'D';
            break;
        case GPIOE_BASE:
            portName = 'E';
            break;
        case GPIOF_BASE:
            portName = 'F';
            break;
        case GPIOG_BASE:
            portName = 'G';
            break;
        case GPIOH_BASE:
            portName = 'H';
            break;
        case GPIOI_BASE:
            portName = 'I';
            break;
        case GPIOJ_BASE:
            portName = 'J';
            break;
        case GPIOK_BASE:
            portName = 'K';
            break;
        default:
            portName = '?';
            break;
    }

    printf("Testing input GPIO P%c%d; reset board when satisfied\n", portName,
           pin.getNumber());

    while (1)
    {
        printf("value P%c%d: %d\n", portName, pin.getNumber(), pin.value());
        Thread::sleep(100);
    }
}
