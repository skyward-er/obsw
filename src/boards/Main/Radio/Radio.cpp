/* Copyright (c) 2024 Skyward Experimental Rocketry
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

#include "Radio.h"

#include <Main/Buses.h>
#include <Main/Sensors/Sensors.h>
#include <common/Radio.h>
#include <radio/SX1278/SX1278Frontends.h>

using namespace Main;
using namespace Boardcore;
using namespace miosix;
using namespace Common;

SX1278Fsk* gRadio{nullptr};

void handleDioIRQ()
{
    SX1278Fsk* instance = gRadio;
    if (instance)
    {
        instance->handleDioIRQ();
    }
}

void setIRQRadio(SX1278Fsk* radio)
{
    FastInterruptDisableLock dl;
    gRadio = radio;
}

void __attribute__((used)) MIOSIX_RADIO_DIO0_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO1_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO3_IRQ() { handleDioIRQ(); }

bool Radio::isStarted() { return started; }

bool Radio::start()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Setup the frontend
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<Skyward433Frontend>();

    // Setup transceiver
    radio = std::make_unique<SX1278Fsk>(
        modules.get<Buses>()->getRadio(), radio::cs::getPin(),
        radio::dio0::getPin(), radio::dio1::getPin(), radio::dio3::getPin(),
        SPI::ClockDivider::DIV_64, std::move(frontend));

    // Store the global radio instance
    setIRQRadio(radio.get());

    // Initialize radio
    auto result = radio->init(MAIN_RADIO_CONFIG);
    if (result != SX1278Fsk::Error::NONE)
    {
        LOG_ERR(logger, "Failed to initialize Main radio");
        return false;
    }

    // Initialize mavdriver
    mavDriver = std::make_unique<MavDriver>(
        radio.get(),
        [this](MavDriver*, const mavlink_message_t& msg)
        { handleMessage(msg); },
        Config::Radio::MAV_SLEEP_AFTER_SEND,
        Config::Radio::MAV_OUT_BUFFER_MAX_AGE);

    if (!mavDriver->start())
    {
        LOG_ERR(logger, "Failed to initialize Main mav driver");
        return false;
    }

    if (scheduler.addTask([this]() { sendHighRateTelemetry(); },
                          Config::Radio::TELEMETRY_PERIOD) == 0)
    {
        LOG_ERR(logger, "Failed to add periodic telemetry task");
        return false;
    }

    if (scheduler.addTask([this]() { sendLowRateTelemetry(); },
                          Config::Radio::TELEMETRY_PERIOD * 2) == 0)
    {
        LOG_ERR(logger, "Failed to add periodic telemetry task");
        return false;
    }

    started = true;
    return true;
}

Boardcore::MavlinkStatus Radio::getMavStatus()
{
    return mavDriver->getStatus();
}

void Radio::enqueuePacket(const mavlink_message_t& msg)
{
    queuedPackets.put(msg);
}

void Radio::flushPackets()
{
    // Flush all packets of the queue
    size_t count = queuedPackets.count();
    for (size_t i = 0; i < count; i++)
    {
        try
        {
            mavDriver->enqueueMsg(queuedPackets.pop());
        }
        catch (...)
        {
            // This shouldn't happen, but still try to prevent it
        }
    }
}

void Radio::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                            Config::Radio::MAV_COMPONENT_ID, &ackMsg, msg.msgid,
                            msg.seq);
    enqueuePacket(ackMsg);
}

void Radio::sendNack(const mavlink_message_t& msg)
{
    mavlink_message_t nackMsg;
    mavlink_msg_nack_tm_pack(Config::Radio::MAV_SYSTEM_ID,
                             Config::Radio::MAV_COMPONENT_ID, &nackMsg,
                             msg.msgid, msg.seq);
    enqueuePacket(nackMsg);
}

void Radio::handleMessage(const mavlink_message_t& msg)
{
    LOG_INFO(logger, "Radio received data");

    sendAck(msg);
}

void Radio::sendHighRateTelemetry()
{
    mavlink_message_t msg;
    packSystemTm(SystemTMList::MAV_FLIGHT_ID, msg);
    enqueuePacket(msg);
    flushPackets();
}

void Radio::sendLowRateTelemetry()
{
    mavlink_message_t msg;
    packSystemTm(SystemTMList::MAV_STATS_ID, msg);
    enqueuePacket(msg);
}

bool Radio::packSystemTm(uint8_t tmId, mavlink_message_t& msg)
{
    ModuleManager& modules = ModuleManager::getInstance();

    switch (tmId)
    {
        case SystemTMList::MAV_FLIGHT_ID:
        {

            mavlink_rocket_flight_tm_t tm;

            Sensors* sensors = modules.get<Sensors>();
            auto imu         = sensors->getLSM6DSRXLastSample();
            auto mag         = sensors->getLIS2MDLLastSample();

            tm.acc_x  = imu.accelerationX;
            tm.acc_y  = imu.accelerationY;
            tm.acc_z  = imu.accelerationZ;
            tm.gyro_x = imu.angularSpeedX;
            tm.gyro_y = imu.angularSpeedY;
            tm.gyro_z = imu.angularSpeedZ;
            tm.mag_x  = mag.magneticFieldX;
            tm.mag_y  = mag.magneticFieldY;
            tm.mag_z  = mag.magneticFieldZ;

            mavlink_msg_rocket_flight_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                                Config::Radio::MAV_COMPONENT_ID,
                                                &msg, &tm);
            return true;
        }
        case SystemTMList::MAV_STATS_ID:
        {
            mavlink_rocket_stats_tm_t tm;

            mavlink_msg_rocket_stats_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                               Config::Radio::MAV_COMPONENT_ID,
                                               &msg, &tm);
            return true;
        }
        default:
            return false;
    }
}
