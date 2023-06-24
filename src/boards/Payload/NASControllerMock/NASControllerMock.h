/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Raul Radu
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

/****************************************************************/
/* !!!! THIS IS A TEMPORARY CLASS DO NOT USE IN PRODUCTION !!!! */
/****************************************************************/

#pragma once

#include <algorithms/NAS/NAS.h>
#include <common/Events.h>
#include <events/FSM.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Payload
{

class NASController : public Boardcore::FSM<NASController>,
                      public Boardcore::Module
{
public:
    NASController();
    ~NASController();

    void state_end(const Boardcore::Event &event);
    bool startModule();
    virtual Boardcore::NASState getNasState();
};

}  // namespace Payload
