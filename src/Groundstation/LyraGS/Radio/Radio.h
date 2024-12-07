/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <Groundstation/Common/HubBase.h>
#include <Groundstation/Common/Ports/Serial.h>
#include <Groundstation/Common/Radio/RadioBase.h>
#include <Groundstation/LyraGS/BoardStatus.h>
#include <Groundstation/LyraGS/Buses.h>
#include <interfaces-impl/hwmapping.h>
#include <radio/SX1278/SX1278Frontends.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace LyraGS
{

class BoardStatus;

class RadioMain : public Boardcore::InjectableWithDeps<
                      Boardcore::InjectableBase<Groundstation::RadioBase>,
                      Buses, BoardStatus>
{
public:
    [[nodiscard]] bool start();

    explicit RadioMain(bool hasBackup) : hasBackup{hasBackup} {};

    RadioMain() : hasBackup{false} {};

    RadioMain(bool hasBackup, bool txEnable)
        : hasBackup{hasBackup}, txEnable{txEnable} {};

    /**
     * @brief Send a mavlink message through this radio if it has been enabled
     * by dipSwitch for transmission
     *
     * @returns false when the queue is full or if tx is not enabled.
     */
    bool sendMsg(const mavlink_message_t& msg);

private:
    bool hasBackup = false;
    bool txEnable  = true;
};
class RadioPayload : public Boardcore::InjectableWithDeps<
                         Boardcore::InjectableBase<Groundstation::RadioBase>,
                         Buses, BoardStatus>
{
public:
    [[nodiscard]] bool start();

    explicit RadioPayload(bool hasBackup) : hasBackup{hasBackup} {};

    RadioPayload() : hasBackup{false} {};

    RadioPayload(bool hasBackup, bool txEnable)
        : hasBackup{hasBackup}, txEnable{txEnable} {};

    /**
     * @brief Send a mavlink message through this radio if it has been enabled
     * by dipSwitch for transmission
     *
     * @returns false when the queue is full or if tx is not enabled.
     */
    bool sendMsg(const mavlink_message_t& msg);

private:
    bool hasBackup = false;
    bool txEnable  = true;
};

}  // namespace LyraGS
