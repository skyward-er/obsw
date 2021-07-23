/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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
#include <stdio.h>

int main()
{
    printf("\nClock configuration : 0x%x \n\n", (unsigned int)RCC->CR);

    // check if HSE is enabled
    if (RCC->CR & RCC_CR_HSEON)
    {
        printf("HSE is on ... ok \n");
    }
    else
    {
        printf("HSE is off ... error \n");
    }

    // check if HSE is ready
    if (RCC->CR & RCC_CR_HSERDY)
    {
        printf("HSE is ready ... ok \n");
    }
    else
    {
        printf("HSE not ready ... error \n");
    }

    // check if HSE is bypassed
    if (RCC->CR & RCC_CR_HSEBYP)
    {
        printf("HSE is bypassed ... error \n");
    }
    else
    {
        printf("HSE is not bypassed ... ok \n");
    }

    // check if Clock Security System is enabled
    if (RCC->CR & RCC_CR_CSSON)
    {
        printf("CSS is on \n");
    }
    else
    {
        printf("CSS is off \n");
    }

    // Wait for the user to press ENTER or the timer to elapse
    printf("Press any key to exit the external oscillator test\n");
    (void)getchar();

    return 0;
}
