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

#include <Main/Radio.h>
#include <Configs/XbeeConfig.h>
#include <drivers/spi/SPIDriver.h>
#include <drivers/Xbee/ATCommands.h>

using std::bind;
using namespace std::placeholders;
using namespace Boardcore;

namespace ParafoilTestDev
{
    /**
    * PUBLIC METHODS
    */
    Radio::Radio(SPIBusInterface& xbee_bus) : xbee_bus(xbee_bus)
    {
        //Create a SPI configuration object
        SPIBusConfig config{};

        //Add a clock divider config
        config.clockDivider = SPI::ClockDivider::DIV_16;

        //Instantiate the xbee object
        xbee = new Xbee::Xbee(xbee_bus, config,
                              XBEE_CS,
                              XBEE_ATTN,
                              XBEE_RESET);
        
        //Set the frame receive callback
        xbee -> setOnFrameReceivedListener(
                bind(&Radio::onXbeeFrameReceived, this, _1));

        //Set the data rate
        Xbee::setDataRate(*xbee, XBEE_80KBPS_DATA_RATE, XBEE_TIMEOUT);

        //TODO Create the mavlink driver
        
        
    }

    Radio::~Radio()
    {

    }

    bool Radio::start()
    {
        return false;
    }

    void Radio::logStatus()
    {

    }

    /**
     * PRIVATE METHODS
     */
    void Radio::onXbeeFrameReceived(Xbee::APIFrame& frame)
    {

    }
}