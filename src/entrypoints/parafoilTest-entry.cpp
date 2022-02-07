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

using namespace ParafoilTestDev;

int main()
{
	miosix::GpioPin servo(GPIOA_BASE, 6);
	servo.mode(miosix::Mode::ALTERNATE);
	servo.alternateFunction(9);

	miosix::GpioPin servo2(GPIOA_BASE, 7);
	servo2.mode(miosix::Mode::ALTERNATE);
	servo2.alternateFunction(9);

	//TODO integrate all the logging stuff
	ParafoilTest::getInstance().start();

	miosix::Thread::sleep(2000);
	ParafoilTest::getInstance().wingController->stop();
	ParafoilTest::getInstance().wingController->start();

	while(true)
	{}

	return 0;
} 
