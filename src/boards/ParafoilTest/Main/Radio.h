/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
#pragma once

#include <TelemetriesTelecommands/Mavlink.h>
#include <drivers/Xbee/Xbee.h>

/**
 * @brief This class defines the interactions beetween the radio module (xbee)
 * and the mavlink messages enumeration. It is used as a main class about these radio 
 * sub modules such as TMRepository and TMCController
 */

namespace ParafoilTestDev
{
    class Radio
    {
    public:
        
        /**
         * @brief The xbee module driver
         */
        Xbee::Xbee* xbee;

        /**
         * @brief The mavlink driver
         */
        MavDriver* mav_driver;

        /**
         * @brief Construct a new Radio object
         * 
         * @param xbee_bus The Xbee SPI bus
         */
        Radio(SPIBusInterface& xbee_bus);

        /**
         * @brief Destroy the Radio object
         */
        ~Radio();

        bool start();

        void logStatus();

    private:
        
        /**
         * @brief Radio frame received callback method.
         * It's used for logging purposes
         */
        void onXbeeFrameReceived(Xbee::APIFrame& frame);

        /**
         * @brief SPI bus
         */
        SPIBusInterface& xbee_bus;
    };
}