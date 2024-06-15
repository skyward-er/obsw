/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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

/**
 * @brief Dip switch status for the GS board
 */
struct DipStatus
{
    bool isARP;
    bool hasBackup;
    bool ip0;
    bool ip1;
    bool ip2;
    bool ip3;
    bool ip4;
    bool ip5;
};

/**
 * @brief Dip switch driver to read the current status of the switch
 */
class DipReader
{
public:
    static DipStatus readDip()
    {
        DipStatus dipReading;

        // Write to the shift register (CS == Not LD)
        miosix::dipSwitch::cs::low();
        miosix::dipSwitch::clk::high();
        miosix::delayUs(100);
        miosix::dipSwitch::clk::low();
        miosix::dipSwitch::cs::high();
        miosix::delayUs(5);

        // Read first register GS(0)/ARP(1)
        dipReading.isARP     = readBit();
        dipReading.hasBackup = readBit();
        dipReading.ip0       = readBit();
        dipReading.ip1       = readBit();
        dipReading.ip2       = readBit();
        dipReading.ip3       = readBit();
        dipReading.ip4       = readBit();
        dipReading.ip5       = readBit();
        dipReading.ip6       = readBit();
        dipReading.ip7       = readBit();
    }

private:
    bool readBit()
    {
        bool bit;
        miosix::dipSwitch::clk::high();
        miosix::delayUs(5);
        bit = miosix::dipSwitch::qh;
        miosix::delayUs(5);
        miosix::dipSwitch::clk::low();
        miosix::delayUs(5);
        return bit;
    }
};