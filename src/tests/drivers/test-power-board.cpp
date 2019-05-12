/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <miosix.h>
#include <Common.h>
#include <interfaces-impl/hwmapping.h>

using namespace miosix;
//using namespace DeathStackBoard;

int main()
{

    using namespace actuators;
    tcPwm::mode(Mode::OUTPUT);

    thCut1::ena::mode(Mode::OUTPUT);
    thCut1::ena::low();
    thCut1::csens::mode(Mode::INPUT_ANALOG);

    thCut2::ena::mode(Mode::OUTPUT);
    thCut2::ena::low();
    thCut2::csens::mode(Mode::INPUT_ANALOG);

    char c;
    while(true)
    {
        scanf("%c", &c);

        switch(c)
        {
            case 'a':
                thCut1::ena::high();
                printf("thCut1 ena\n");
                break;
            case 'b':
                thCut2::ena::high();
                printf("thCut2 ena\n");
                break;
            case 'c':
                tcPwm::high();
                printf("tcPwm\n");
                break;
            case 'd':
                thCut1::ena::low();
                thCut2::ena::low();
                tcPwm::low();
                printf("Closing everything\n");
                break;
            default:
                 break;
        }
    }
}