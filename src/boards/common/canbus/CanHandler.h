/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

#include <ActiveObject.h>
#include <common/canbus/CanConfig.h>
#include <common/canbus/MockSensors/MockSensor.h>
#include <drivers/canbus/CanProtocol.h>
#include <drivers/canbus/Canbus.h>

#include <functional>

namespace common
{

/**
 * @brief Simple Struct for filters, to accept all incoming messages do not
 * change any value
 *
 */
struct Filter
{
    int8_t source      = -1;
    int8_t destination = -1;
};

/**
 * @brief Canbus high level implementation.
 *
 * This class takes care of creating the ID of the CanData, sending it to
 * CanProtocol and receiving incoming packets calling the right events.
 */
class CanHandler
{
private:
    Boards source;
    Boardcore::Canbus::CanProtocol *can;
    MockSensor *sensors[NumberOfSensor];
    uint8_t numberOfObservedSensor = 0;
    miosix::FastMutex mutex;
    Boardcore::Canbus::CanbusDriver *canPhy;

public:
    /**
     * @brief Construct a new CanHandler object
     *
     * @param source: the Id of the sending board.
     */
    explicit CanHandler(Boards source);

    /**
     * @brief Init the physical canbus and start CanProtocol.
     *
     * @warning You can only add filters before calling this function
     */
    void startHandler();

    /**
     * @brief Adds filter to the physical canbus .
     *
     * @param filter: The filter you want to apply
     * @warning You can only add filters before calling startHandler()
     */
    bool addFilter(Filter filter);

    /**
     * @brief Adds a MockSensor to the list of watchedSensor .
     *
     * @param newSensor: The sensor to add to the watch-list
     */
    bool addMock(MockSensor *newSensor);

    /**
     * @brief Calculate the id of the packet and calls the send function of
     * CanProtocol.
     *
     * @param destination: The Id of the destination board.
     * @param p: The priority of the packet.
     * @param t: The type of the packet sent.
     * @param idT: The IDtype of the packet sent.
     * @param toSend: The packet to be sent.
     */
    void sendCan(Boards destination, Priority p, Type t, uint8_t idT,
                 Boardcore::Canbus::CanData toSend);

    /**
     * @brief Calls sendData when we want to send a event (without creating a
     * CanData).
     *
     * @param destination: The Id of the destination board.
     * @param p: The priority of the packet.
     * @param t: The type of the packet sent.
     * @param idT: The IDtype of the packet sent.
     */
    void sendCan(Boards destination, Priority p, Type t, uint8_t idT);

    ~CanHandler();

protected:
    /**
     * @brief Function passe to CanProtocol called once a CanData message is
     * created, based on the id calls the right event or puts the data in the
     * right sensor is received we update data and send events as needed
     *
     * @param data: The CanData message to decode
     * @warning Execution time should be kept to a minimum
     */
    void callback(Boardcore::Canbus::CanData data);
};

}  // namespace common
