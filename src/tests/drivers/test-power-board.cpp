/*
 * Copyright (c) 2018-2021 Skyward Experimental Rocketry
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

    using namespace actuators::nosecone;
    using namespace actuators::airbrakes;

    char c;

    while(true)
    {
        printf("1 - Nosecone servo PWM \n");
        printf("2 - Thermal cutter PWM \n");
        printf("3 - Thermal cutter 1 enable \n");
        printf("4 - Thermal cutter 2 enable \n");
        printf("5 - Aerobrakes servo PWM \n");
        printf("6 - Turn-off everything \n");
        printf(">> ");

        scanf("%c", &c);

        switch(c)
        {
            case '1':
                nc_servo_pwm::high();
                printf("nosecone servo pwm\n");
                break;
            case '2':
                th_cut_pwm::high();
                printf("thermal cutter pwm\n");
                break;
            case '3':
                thCut1::ena::high();
                printf("thCut1 ena\n");
                break;
            case '4':
                thCut2::ena::high();
                printf("thCut2 ena\n");
                break;
            case '5':
                airbrakes_servo_pwm::high();
                printf("aerobrakes servo pwm\n");
                break;
            case '6':
                nc_servo_pwm::high();
                thCut1::ena::low();
                thCut2::ena::low();
                th_cut_pwm::low();
                printf("Closing everything\n");
                break;
            default:
                 break;
        }
    }
}