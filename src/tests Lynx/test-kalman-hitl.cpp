/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Mozzarelli
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

#include <EventClasses.h>
#include <events/EventBroker.h>
#include <events/Events.h>
#include <events/FSM.h>

#include <iostream>
#include <string>

#include "ADA/ADAController.h"

typedef miosix::Gpio<GPIOG_BASE, 13> greenLed;
typedef miosix::Gpio<GPIOG_BASE, 14> redLed;
using namespace DeathStackBoard;
ADAController ada;
void handleCommands(std::string line);

int main()
{
    std::cout << "A_INIT: \n" << A_INIT;

    // Set led pin modes
    {
        miosix::FastInterruptDisableLock dLock;
        greenLed::mode(miosix::Mode::OUTPUT);
        redLed::mode(miosix::Mode::OUTPUT);
    }

    // Start event broker and ada
    sEventBroker.start();
    ada.start();

    // Read serial
    std::string line;
    while (std::getline(std::cin, line))
    {
        if (line[0] == '#')
        {
            handleCommands(line);
        }
    }
}

void handleCommands(std::string line)
{
    // Extract command name
    unsigned delimiterIndex = line.find_last_of("#");
    std::string command     = line.substr(1, delimiterIndex - 1);

    // TRACE("ADA HITL Test command: %s \n", command.c_str());

    // Blink green led: command received
    greenLed::high();
    miosix::Thread::sleep(50);
    greenLed::low();

    if (command == "INPUT_SAMPLE")
    {
        // char* end;
        // std::string height = line.substr(delimiterIndex+1,
        // std::string::npos); float y = strtof(height.c_str(), &end);
        // TODO: Change to accept temperature
        // ada.updateAltitude(y);
    }
    else if (command == "GET_STATE")
    {
        std::cout << "#X0#" << ada.getKalmanState().x0 << "\n";
        std::cout << "#X1#" << ada.getKalmanState().x1 << "\n";
        std::cout << "#X2#" << ada.getKalmanState().x2 << "\n";
    }
    // TODO: Change to new version
    // else if ( command == "GET_CALIB" )
    // {
    //     std::cout << "#N_SAMPLES#" << ada.getCalibrationData().stats.nSamples
    //     << "\n"; std::cout << "#AVG#" << ada.getCalibrationData().stats.mean
    //     << "\n";
    // }
    else if (command == "EV_TC_RESET_CALIBRATION")
    {
        sEventBroker.post({EV_TC_RESET_CALIBRATION}, TOPIC_ADA);
    }
    else if (command == "EV_LIFTOFF")
    {
        sEventBroker.post({EV_LIFTOFF}, TOPIC_FLIGHT_EVENTS);
    }
    else if (command == "EV_APOGEE")
    {
        sEventBroker.post({EV_APOGEE}, TOPIC_FLIGHT_EVENTS);
    }
    else if (command == "EV_SET_DPL_ALTITUDE")
    {
        int alt = std::atoi(line.substr(20, std::string::npos).c_str());
        DeploymentAltitudeEvent ev;
        ev.dplAltitude = alt;
        ev.sig         = EV_TC_SET_DPL_ALTITUDE;
        sEventBroker.post(ev, TOPIC_TC);
    }
    else if (command == "EV_DPL_ALTITUDE")
    {
        sEventBroker.post({EV_DPL_ALTITUDE}, TOPIC_FLIGHT_EVENTS);
    }
    else
    {
        // Blink red led: command not recognized
        redLed::high();
        miosix::Thread::sleep(50);
        redLed::low();
    }
}
