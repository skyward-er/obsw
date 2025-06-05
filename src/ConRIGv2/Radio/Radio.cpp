/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Ettore Pane, Niccolò Betto
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

#include <common/MavlinkOrion.h>
#include <common/Radio.h>
#include <diagnostic/SkywardStack.h>
#include <drivers/interrupt/external_interrupts.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>
#include <radio/SX1278/SX1278Frontends.h>

#include <chrono>
#include <thread>

using namespace std::chrono;
using namespace miosix;
using namespace Boardcore;
using namespace ConRIGv2;
using namespace Common;
using namespace Boardcore::Units::Frequency;

SX1278Lora* gRadio{nullptr};

void handleDioIRQ()
{
    SX1278Lora* instance = gRadio;
    if (instance)
        instance->handleDioIRQ();
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
            bool wasArmed   = isArmed;
            bool isNowArmed = armingState == 1;
            isArmed         = isNowArmed;

            // Reset the message counter to a value higher than zero to avoid
            // long pauses without audio feedback after disarming
            if (wasArmed && !isNowArmed)
                messagesReceived = Config::Radio::AUDIO_FEEDBACK_RESET_VALUE;
            else
                messagesReceived += 1;

            if (isNowArmed)
                getModule<Buttons>()->enableIgnition();
            else
                getModule<Buttons>()->disableIgnition();

            break;
        }

        case MAVLINK_MSG_ID_ACK_TM:
        {
            int id = mavlink_msg_ack_tm_get_recv_msgid(&msg);
            // we assume this ack is about the last sent message
            if (id == MAVLINK_MSG_ID_CONRIG_STATE_TC)
            {
                // Reset the internal button state
                resetButtonState();

                if (pingThread)
                    pingThread->wakeup();
            }

            break;
        }
    }

    getModule<Hub>()->dispatchToPorts(msg);
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

void Radio::buzzerOn()
{
    buzzer.enableChannel(TimerUtils::Channel::MIOSIX_BUZZER_CHANNEL);
}

void Radio::buzzerOff()
{
    buzzer.disableChannel(TimerUtils::Channel::MIOSIX_BUZZER_CHANNEL);
}

void Radio::buzzerTask()
{
    if ((!isArmed &&
         messagesReceived > Config::Radio::AUDIO_FEEDBACK_THRESHOLD) ||
        (isArmed && messagesReceived > 0))
    {
        messagesReceived = 0;
        buzzerOn();
    }
    else
    {
        buzzerOff();
    }
}

void Radio::updateButtonState(const mavlink_conrig_state_tc_t& state)
{
    Lock<FastMutex> lock{buttonsMutex};
    // Merge the new state with the old one, an extra-dumb way to ensure
    // that we don't lose any button pressess
    buttonState.ox_filling_btn |= state.ox_filling_btn;
    buttonState.ox_release_btn |= state.ox_release_btn;
    buttonState.n2_filling_btn |= state.n2_filling_btn;
    buttonState.n2_release_btn |= state.n2_release_btn;
    buttonState.n2_detach_btn |= state.n2_detach_btn;
    buttonState.ox_venting_btn |= state.ox_venting_btn;
    buttonState.nitrogen_btn |= state.nitrogen_btn;
    buttonState.ox_detach_btn |= state.ox_detach_btn;
    buttonState.n2_quenching_btn |= state.n2_quenching_btn;
    buttonState.ignition_btn |= state.ignition_btn;

    // Don't merge lever states
    buttonState.n2_3way_switch = state.n2_3way_switch;
    buttonState.tars_switch    = state.tars_switch;
    buttonState.arm_switch     = state.arm_switch;
}

void Radio::resetButtonState()
{
    Lock<FastMutex> lock{buttonsMutex};
    // Save and restore lever states
    auto n2_3way_switch        = buttonState.n2_3way_switch;
    auto tars_switch           = buttonState.tars_switch;
    auto arm_switch            = buttonState.arm_switch;
    buttonState                = {};
    buttonState.n2_3way_switch = n2_3way_switch;
    buttonState.tars_switch    = tars_switch;
    buttonState.arm_switch     = arm_switch;
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
        radio.get(), [this](MavDriver*, const mavlink_message_t& msg)
        { handleMessage(msg); }, Config::Radio::MAV_SLEEP_AFTER_SEND,
        Config::Radio::MAV_OUT_BUFFER_MAX_AGE);

    if (!mavDriver->start())
    {
        LOG_ERR(logger, "Failed to initialize ConRIGv2 mav driver");
        return false;
    }

    // Periodic telemetry ping thread
    pingThread = Thread::create(
        [](void* pRadio)
        {
            auto* radio = static_cast<Radio*>(pRadio);

            while (true)
            {
                radio->sendPeriodicPing();

                auto wakeup = steady_clock::now() + Config::Radio::PING_TIMEOUT;

                // Wait until signaled (ack received) or timeout
                Thread::getCurrentThread()->timedWait(
                    wakeup.time_since_epoch().count());
            }
        },
        miosix::STACK_DEFAULT_FOR_PTHREAD,
        getModule<BoardScheduler>()->radioPriority(), this,
        miosix::Thread::DEFAULT);

    auto& buzzzerSched = getModule<BoardScheduler>()->buzzer();
    buzzzerSched.addTask([this]() { buzzerTask(); }, 20_hz,
                         TaskScheduler::Policy::RECOVER);

    return true;
}

MavlinkStatus Radio::getMavlinkStatus() { return mavDriver->getStatus(); }

Radio::Radio() : buzzer(MIOSIX_BUZZER_TIM, 523)
{
    buzzer.setDutyCycle(TimerUtils::Channel::MIOSIX_BUZZER_CHANNEL, 0.5);
}
