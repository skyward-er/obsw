/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Giacomo Caironi
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

#include <common/Mavlink.h>
#include <common/Radio.h>
#include <diagnostic/SkywardStack.h>
#include <drivers/interrupt/external_interrupts.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>
#include <radio/SX1278/SX1278Frontends.h>

#include <thread>

using namespace miosix;
using namespace Boardcore;
using namespace ConRIG;
using namespace Common;

SX1278Lora* gRadio{nullptr};

void handleDioIRQ()
{
    SX1278Lora* instance = gRadio;
    if (instance)
    {
        instance->handleDioIRQ();
    }
}

void setIRQRadio(SX1278Lora* radio)
{
    FastInterruptDisableLock dl;
    gRadio = radio;
}

void __attribute__((used)) MIOSIX_RADIO_DIO0_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO1_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO3_IRQ() { handleDioIRQ(); }

void Radio::handleMessage(const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_GSE_TM:
        {
            int armingState = mavlink_msg_gse_tm_get_arming_state(&msg);
            messagesReceived += 1;

            isArmed = armingState == 1;
            if (armingState == 1)
            {
                getModule<Buttons>()->enableIgnition();
            }
            else
            {
                getModule<Buttons>()->disableIgnition();
            }

            break;
        }

        case MAVLINK_MSG_ID_ACK_TM:
        {
            int id = mavlink_msg_ack_tm_get_recv_msgid(&msg);
            // we assume this ack is about the last sent message
            if (id == MAVLINK_MSG_ID_CONRIG_STATE_TC)
            {
                Lock<FastMutex> lock{buttonsMutex};
                // Reset the internal button state
                buttonState.ignition_btn         = false;
                buttonState.filling_valve_btn    = false;
                buttonState.venting_valve_btn    = false;
                buttonState.release_pressure_btn = false;
                buttonState.quick_connector_btn  = false;
                buttonState.start_tars_btn       = false;
                buttonState.arm_switch           = false;
            }

            break;
        }
    }

    getModule<Serial>()->sendMessage(msg);
}

bool Radio::enqueueMessage(const mavlink_message_t& msg)
{
    Lock<FastMutex> lock{queueMutex};
    if (messageQueue.isFull())
    {
        return false;
    }
    else
    {
        messageQueue.put(msg);
        return true;
    }
}

void Radio::sendPeriodicPing()
{
    mavlink_message_t msg;

    {
        Lock<FastMutex> lock{buttonsMutex};
        mavlink_msg_conrig_state_tc_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &msg, &buttonState);
    }

    // Flush the queue
    {
        Lock<FastMutex> lock{queueMutex};
        // TODO(davide.mor): Maybe implement a maximum per ping?
        for (size_t i = 0; i < messageQueue.count(); i++)
        {
            try
            {
                mavDriver->enqueueMsg(messageQueue.pop());
            }
            catch (...)
            {
                // This shouldn't happen, but still try to prevent it
            }
        }
    }

    // Finally make sure we always send the periodic ping
    mavDriver->enqueueMsg(msg);
}

void Radio::loopBuzzer()
{
    while (true)
    {
        Thread::sleep(100);
        // Doesn't matter the precision, the important thing is
        // the beep, this is because an atomic is used
        if ((!isArmed && messagesReceived > 2) ||
            (isArmed && messagesReceived > 0))
        {
            messagesReceived = 0;
            ui::buzzer::low();
            Thread::sleep(100);
            ui::buzzer::high();
        }
    }
}

void Radio::setButtonsState(mavlink_conrig_state_tc_t state)
{
    Lock<FastMutex> lock{buttonsMutex};
    // The OR operator is introduced to make sure that the receiver
    // understood the command
    buttonState.ignition_btn |= state.ignition_btn;
    buttonState.filling_valve_btn |= state.filling_valve_btn;
    buttonState.venting_valve_btn |= state.venting_valve_btn;
    buttonState.release_pressure_btn |= state.release_pressure_btn;
    buttonState.quick_connector_btn |= state.quick_connector_btn;
    buttonState.start_tars_btn |= state.start_tars_btn;
    buttonState.arm_switch |= state.arm_switch;
}

bool Radio::start()
{
    // Setup the frontend
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<EbyteFrontend>(radio::txEn::getPin(),
                                        radio::rxEn::getPin());

    // Setup transceiver
    radio = std::make_unique<SX1278Lora>(
        getModule<Buses>()->getRadio(), radio::cs::getPin(),
        radio::dio0::getPin(), radio::dio1::getPin(), radio::dio3::getPin(),
        SPI::ClockDivider::DIV_64, std::move(frontend));

    // Store the global radio instance
    setIRQRadio(radio.get());

    // Initialize radio
    auto result = radio->init(RIG_RADIO_CONFIG);
    if (result != SX1278Lora::Error::NONE)
    {
        LOG_ERR(logger, "Failed to initialize RIG radio");
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
        LOG_ERR(logger, "Failed to initialize ConRIG mav driver");
        return false;
    }

    TaskScheduler& scheduler = getModule<BoardScheduler>()->getRadioScheduler();

    if (scheduler.addTask([this]() { sendPeriodicPing(); },
                          Config::Radio::PING_GSE_PERIOD,
                          TaskScheduler::Policy::RECOVER) == 0)
    {
        LOG_ERR(logger, "Failed to add ping task");
        return false;
    }

    buzzerLooper = std::thread([this]() { loopBuzzer(); });

    return true;
}

MavlinkStatus Radio::getMavlinkStatus() { return mavDriver->getStatus(); }

Radio::Radio()
{
    buttonState.ignition_btn         = false;
    buttonState.filling_valve_btn    = false;
    buttonState.venting_valve_btn    = false;
    buttonState.release_pressure_btn = false;
    buttonState.quick_connector_btn  = false;
    buttonState.start_tars_btn       = false;
    buttonState.arm_switch           = false;
}