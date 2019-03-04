/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <cstdio>

#include "boards/Homeone/Events.h"
#include "boards/Homeone/IgnitionController/IgnitionController.h"
#include "boards/Homeone/Canbus/CanProxy.h"

using namespace miosix;
using namespace HomeoneBoard;

int main()
{
	CanManager* can_mgr = new CanManager(CAN1);
    CanProxy* can   = new CanProxy(can_mgr);

    IgnitionController* ctrl = new IgnitionController(can);
    
    ctrl->getStatus();
    
    for(;;)
    {
        printf("end\n");
    }
	return 0;
}
