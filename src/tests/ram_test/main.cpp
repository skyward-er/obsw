 /***************************************************************************
  *   Copyright (C) 2012 by Terraneo Federico                               *
  *                                                                         *
  *   This program is free software; you can redistribute it and/or modify  *
  *   it under the terms of the GNU General Public License as published by  *
  *   the Free Software Foundation; either version 2 of the License, or     *
  *   (at your option) any later version.                                   *
  *                                                                         *
  *   This program is distributed in the hope that it will be useful,       *
  *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
  *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
  *   GNU General Public License for more details.                          *
  *                                                                         *
  *   You should have received a copy of the GNU General Public License     *
  *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
  ***************************************************************************/

// RAM testing code
// Useful to test the external RAM of a board.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "sha1.h"
#include <miosix.h>
#include <stdexcept>

const unsigned int ramBase=0x60000000; //Tune this to the right value
const unsigned int ramSize=524288;     //Tune this to the right value

bool shaCmp(unsigned int a[5], unsigned int b[5])
{
    if(memcmp(a,b,20)==0) return false;
    iprintf("Mismatch\n");
    for(int i=0;i<5;i++) iprintf("%04x",a[i]); iprintf("\n");
    for(int i=0;i<5;i++) iprintf("%04x",b[i]); iprintf("\n");
    return true;
}

template<typename T> bool ramTest()
{
    SHA1 sha;
    sha.Reset();
    srand(0x7ad3c099*sizeof(T)); //Just to shuffle initialization between tests
    for(unsigned int i=ramBase;i<ramBase+ramSize;i+=sizeof(T))
    {
        T *p=reinterpret_cast<T*>(i);
        T v=static_cast<T>(rand());
        *p=v;
        sha.Input(reinterpret_cast<const unsigned char*>(&v),sizeof(T));
    }
    unsigned int a[5];
    sha.Result(a);
    sleep(10); //To check SDRAM retention ability
    sha.Reset();
    for(unsigned int i=ramBase;i<ramBase+ramSize;i+=sizeof(T))
    {
        T *p=reinterpret_cast<T*>(i);
        T v=*p;
        sha.Input(reinterpret_cast<const unsigned char*>(&v),sizeof(T));
    }
    unsigned int b[5];
    sha.Result(b);
    return shaCmp(a,b);
}


typedef miosix::Gpio<GPIOG_BASE, 2> led1;

int main()
{
    led1::mode(miosix::Mode::OUTPUT);
    led1::low();

    uint64_t n_attempts = 0, n_ok = 0, n_fails = 0;

    FILE* file = fopen("/sd/log.txt","w");

    if (file == NULL)
        throw std::runtime_error("Error opening log file");

    for(;;)
    {
        fprintf(file, "TEST STATUS: attempts: %llx, passed: %llx, failed %llx\n",
                        n_attempts, n_ok, n_fails);
        n_attempts++;

        iprintf("RAM test\nTesting word size transfers\n");
        if(ramTest<unsigned int>()) {
            n_fails++;
            continue;
        }

        iprintf("Testing halfword size transfers\n");
        if(ramTest<unsigned short>()) {
            n_fails++;
            continue;
        }

        iprintf("Testing byte size transfers\n");
        if(ramTest<unsigned char>()) {
            n_fails++;
            continue;
        }

        n_ok++;
        iprintf("Ok\n");

        led1::high();
        miosix::Thread::sleep(200);
        led1::low();
    }
}
