/* Copyright (c) 2019-2023 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio, Federico Mandelli
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

#include "PinHandler.h"

#include <MockupMain/Configs/PinHandlerConfig.h>
#include <common/Events.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>

#include <functional>

using namespace std::placeholders;
using namespace miosix;
using namespace Boardcore;
using namespace Common;

namespace MockupMain
{

void PinHandler::onExpulsionPinTransition(PinTransition transition)
{
    if (transition == NC_DETACH_PIN_TRIGGER)
        EventBroker::getInstance().post(FLIGHT_NC_DETACHED, TOPIC_FLIGHT);
}

bool PinHandler::start()
{
    return true;
}

bool PinHandler::isStarted() { return running; }

std::map<PinsList, PinData> PinHandler::getPinsData()
{
    std::map<PinsList, PinData> data;

    data[PinsList::NOSECONE_PIN] =
        pinObserver->getPinData(inputs::expulsion::getPin());

    return data;
}

PinHandler::PinHandler() : running(false)
{
    pinObserver->registerPinCallback(
        inputs::expulsion::getPin(),
        std::bind(&PinHandler::onExpulsionPinTransition, this, _1),
        NC_DETACH_PIN_THRESHOLD);
}
}  // namespace MockupMain
