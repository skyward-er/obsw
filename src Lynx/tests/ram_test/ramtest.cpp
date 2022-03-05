/* Copyright (c) 2012 Skyward Experimental Rocketry
 * Author: Federico Terraneo
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

// RAM testing code
// Useful to test the external RAM of a board.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "sha1.h"

const unsigned int ramBase = 0xd0000000;       // Tune this to the right value
const unsigned int ramSize = 8 * 1024 * 1024;  // Tune this to the right value

bool shaCmp(unsigned int a[5], unsigned int b[5])
{
    if (memcmp(a, b, 20) == 0)
        return false;
    iprintf("Mismatch\n");
    for (int i = 0; i < 5; i++)
        iprintf("%04x", a[i]);
    iprintf("\n");
    for (int i = 0; i < 5; i++)
        iprintf("%04x", b[i]);
    iprintf("\n");
    return true;
}

template <typename T>
bool ramTest()
{
    SHA1 sha;
    sha.Reset();
    srand(0x7ad3c099 *
          sizeof(T));  // Just to shuffle initialization between tests
    for (unsigned int i = ramBase; i < ramBase + ramSize; i += sizeof(T))
    {
        T *p = reinterpret_cast<T *>(i);
        T v  = static_cast<T>(rand());
        *p   = v;
        sha.Input(reinterpret_cast<const unsigned char *>(&v), sizeof(T));
    }
    unsigned int a[5];
    sha.Result(a);
    sleep(10);  // To check SDRAM retention ability
    sha.Reset();
    for (unsigned int i = ramBase; i < ramBase + ramSize; i += sizeof(T))
    {
        T *p = reinterpret_cast<T *>(i);
        T v  = *p;
        sha.Input(reinterpret_cast<const unsigned char *>(&v), sizeof(T));
    }
    unsigned int b[5];
    sha.Result(b);
    return shaCmp(a, b);
}

int main()
{
    printf("\nBEFORE YOU START:\n");
    printf(
        "1. Have you compiled with the right linker script (e.g. "
        "2m+256k_rom)?\n");
    printf(
        "2. Have you set RamBase and RamSize correctly in this entrypoint?\n");
    printf("3. Have you enable the RAM (aka compile with -D__ENABLE_XRAM)?\n");
    printf("Press enter to start...");
    getchar();

    for (;;)
    {
        iprintf("RAM test\nTesting word size transfers\n");
        if (ramTest<unsigned int>())
            return 1;
        iprintf("Testing halfword size transfers\n");
        if (ramTest<unsigned short>())
            return 1;
        iprintf("Testing byte size transfers\n");
        if (ramTest<unsigned char>())
            return 1;
        iprintf("Ok\n");
    }
}
