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

const unsigned int ramBase=0xd0000000; //Tune this to the right value
const unsigned int ramSize=8*1024*1024;     //Tune this to the right value

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

int main()
{
    printf("\n------- BEFORE YOU START: ------- \n");
    printf("1. Have you read the README.md contained in this same folder?\n");
    printf("2. Have you compiled with the right linker script (e.g. 2m+256k_rom)?\n");
    printf("3. Have you set RamBase and RamSize correctly in this entrypoint?\n");
    printf("4. Have you enabled the RAM (aka compile with -D__ENABLE_XRAM)?\n");
    printf("Press enter to start...");
    getchar();

    for(;;)
    {
        iprintf("RAM test\nTesting word size transfers\n");
        if(ramTest<unsigned int>()) return 1;
        iprintf("Testing halfword size transfers\n");
        if(ramTest<unsigned short>()) return 1;
        iprintf("Testing byte size transfers\n");
        if(ramTest<unsigned char>()) return 1;
        iprintf("Ok\n\n");
    }
}