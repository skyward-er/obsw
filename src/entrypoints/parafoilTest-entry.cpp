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
#include <miosix.h>
#include <ParafoilTest.h>
#include <Configs/XbeeConfig.h>

using namespace ParafoilTestDev;	

int main()
{
	miosix::GpioPin spiSck(GPIOA_BASE, 5);
	miosix::GpioPin spiMiso(GPIOB_BASE, 4);
	miosix::GpioPin spiMosi(GPIOA_BASE, 7);

	spiSck.mode(miosix::Mode::ALTERNATE);
    spiSck.alternateFunction(5);
    spiMiso.mode(miosix::Mode::ALTERNATE);
    spiMiso.alternateFunction(5);
    spiMosi.mode(miosix::Mode::ALTERNATE);
    spiMosi.alternateFunction(5);

	XBEE_CS.mode(miosix::Mode::OUTPUT);
	XBEE_CS.high();

	XBEE_ATTN.mode(miosix::Mode::INPUT);

	XBEE_RESET.mode(miosix::Mode::OUTPUT);

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // Enable SPI1 bus

	//TODO integrate all the logging stuff
	ParafoilTest::getInstance().start();

	while(true)
	{	
		miosix::Thread::sleep(100);
	}

	return 0;
} 
