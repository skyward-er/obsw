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
#include <Configs/SensorsConfig.h>

using namespace ParafoilTestDev;	

void enablePin()
{
	GPS_CS.mode(miosix::Mode::OUTPUT);
	IMU_CS.mode(miosix::Mode::OUTPUT);
	PRESS_CS.mode(miosix::Mode::OUTPUT);

	miosix::GpioPin SCK(GPIOA_BASE, 5);
	miosix::GpioPin MISO(GPIOB_BASE, 4);
	miosix::GpioPin MOSI(GPIOA_BASE, 7);

	SCK.mode(miosix::Mode::ALTERNATE);
	MISO.mode(miosix::Mode::ALTERNATE);
	MOSI.mode(miosix::Mode::ALTERNATE);

	SCK.alternateFunction(5);
	MISO.alternateFunction(5);
	MOSI.alternateFunction(5);

	/*XBEE_SCK.mode(miosix::Mode::ALTERNATE);
    XBEE_SCK.alternateFunction(5);
    XBEE_MISO.mode(miosix::Mode::ALTERNATE);
    XBEE_MISO.alternateFunction(5);
    XBEE_MOSI.mode(miosix::Mode::ALTERNATE);
    XBEE_MOSI.alternateFunction(5);

	XBEE_CS.mode(miosix::Mode::OUTPUT);
	XBEE_CS.high();

	XBEE_ATTN.mode(miosix::Mode::INPUT);

	XBEE_RESET.mode(miosix::Mode::OUTPUT);*/

	GPS_CS.high();
	IMU_CS.high();
	PRESS_CS.high();
}

int main()
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // Enable SPI1 bus
	RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;  // Enable SPI4 bus
	enablePin();

	//TODO integrate all the logging stuff
	ParafoilTest::getInstance().start();
	while(true)
	{	
		miosix::Thread::sleep(100);
	}

	return 0;
} 
