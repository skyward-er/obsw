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

#include <Groundstation/Automated/LogSniffing.h>
#include <Groundstation/Automated/SMA/SMA.h>
#include <Groundstation/Common/HubBase.h>
#include <Groundstation/LyraGS/BoardStatus.h>
#include <Groundstation/LyraGS/Ports/Ethernet.h>
#include <Groundstation/LyraGS/Ports/SerialLyraGS.h>
#include <Groundstation/LyraGS/Radio/Radio.h>
#include <algorithms/NAS/NASState.h>
#include <common/MavlinkLyra.h>
#include <miosix.h>
#include <sensors/SensorData.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace LyraGS
{
class BoardStatus;
}

namespace Antennas
{

/*  This is used to avoid discarding messages in case the rocket timestamp is
 * reset. Therefore if older than the discard msg delay, resets the messages
 * last timestamp */
static constexpr uint64_t DISCARD_MSG_DELAY =
    10 * 1000000;  ///< Maximum time for which the message, if older is
                   ///< discarded. [micros]

/**
 * @brief Central hub connecting all outgoing and ingoing modules.
 */
class Hub : public Boardcore::InjectableWithDeps<
                Boardcore::InjectableBase<Groundstation::HubBase>, SMA,
                LyraGS::RadioMain, LyraGS::RadioPayload, LyraGS::BoardStatus,
                LyraGS::SerialLyraGS, LyraGS::EthernetGS>
{
public:
    /**
     * @brief Dispatch to the correct interface and outgoing packet (gs ->
     * rocket).
     */
    void dispatchOutgoingMsg(const mavlink_message_t& msg) override;

    /**
     * @brief Dispatch to the correct interface and incoming packet (rocket ->
     * gs).
     */
    void dispatchIncomingMsg(const mavlink_message_t& msg) override;

    /**
     * @brief Sends via radio an acknowledge message about the parameter passed
     * message
     */
    void sendAck(const mavlink_message_t& msg);

    /**
     * @brief Synchronized getter for the last rocket origin for NAS.
     */
    bool getRocketOrigin(Boardcore::GPSData& rocketOrigin);

    /**
     * @brief Synchronized getter for the last rocket NAS state.
     *
     * @return true only if the rocket NAS state and is valid and the value is
     * new (got from radio)
     */
    bool getLastRocketNasState(Boardcore::NASState& nasState);

    bool hasNewNasState();

private:
    /**
     * @brief Synchronized setter for the last rocket NAS state.
     */
    void setRocketNasState(const Boardcore::NASState& newRocketNasState);

    /**
     * @brief Synchronized setter for the last rocket origin for NAS.
     */
    void setRocketOrigin(const Boardcore::GPSData& newRocketCoordinates);

    Boardcore::GPSData lastRocketCoordinates;
    bool originReceived = false;
    Boardcore::NASState lastRocketNasState;
    bool rocketNasSet = false;
    miosix::FastMutex coordinatesMutex;
    miosix::FastMutex nasStateMutex;
    bool hasNewNasSet              = false;
    uint64_t lastFlightTMTimestamp = 0;
    uint64_t lastStatsTMTimestamp  = 0;
};

}  // namespace Antennas
