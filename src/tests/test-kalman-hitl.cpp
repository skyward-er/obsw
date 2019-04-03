
#pragma once

#include <iostream>
#include "DeathStack/ADA/ADA.h"
#include <events/FSM.h>
#include <Common.h>
#include <string>

typedef miosix::Gpio<GPIOG_BASE, 13> greenLed;
using namespace DeathStackBoard;
ADA a;
void handleCommands(std::string line);

int main()
{
    {
        miosix::FastInterruptDisableLock dLock;
        greenLed::mode(miosix::Mode::OUTPUT);
    }

    std::string line;
    while ( std::getline(std::cin, line) )
    {
        std::cout << "GOT LINE: " << line << "\n";
        if (line[0] == '#')
        {
            handleCommands(line);
        }
    }
    std::cout << "END" << "\n";

}

void handleCommands(std::string line)
{
    std::cout << "HANDLE COMMAND: " << line << "\n";
    if ( line.substr(1,12) == "INPUT_SAMPLE" )
    {
        char* end;
        float y = strtof(line.substr(12, std::string::npos).c_str(), &end );
        a.update(y);
        std::cout << "UPDATED WITH SAMPLE " << y << "\n";
    }
}