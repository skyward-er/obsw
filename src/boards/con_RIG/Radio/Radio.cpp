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
#include <con_RIG/BoardScheduler.h>
#include <con_RIG/Buses.h>
#include <con_RIG/Buttons/Buttons.h>
#include <diagnostic/SkywardStack.h>
#include <drivers/interrupt/external_interrupts.h>
#include <events/EventBroker.h>
#include <radio/SX1278/SX1278Frontends.h>
#include <radio/Xbee/ATCommands.h>

#include <thread>

using namespace std;
using namespace miosix;
using namespace placeholders;
using namespace Boardcore;
using namespace con_RIG::Config::Radio;

void __attribute__((used)) EXTI1_IRQHandlerImpl()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<con_RIG::Radio>()->transceiver != nullptr)
    {
        modules.get<con_RIG::Radio>()->transceiver->handleDioIRQ();
    }
}

void __attribute__((used)) EXTI12_IRQHandlerImpl()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<con_RIG::Radio>()->transceiver != nullptr)
    {
        modules.get<con_RIG::Radio>()->transceiver->handleDioIRQ();
    }
}

void __attribute__((used)) EXTI13_IRQHandlerImpl()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<con_RIG::Radio>()->transceiver != nullptr)
    {
        modules.get<con_RIG::Radio>()->transceiver->handleDioIRQ();
    }
}

namespace con_RIG
{

void Radio::handleMavlinkMessage(MavDriver* driver,
                                 const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_GSE_TM:
        {
            int arming_state = mavlink_msg_gse_tm_get_arming_state(&msg);
            ModuleManager::getInstance().get<Buttons>()->setRemoteArmState(
                arming_state);
            messageReceived +=
                arming_state == 1
                    ? 10
                    : 2;  // The beep increments in speed as the state is armed
        }
        case MAVLINK_MSG_ID_ACK_TM:
        {
            int id = mavlink_msg_ack_tm_get_recv_msgid(&msg);
            // we assume this ack is about the last sent message
            if (id == MAVLINK_MSG_ID_CONRIG_STATE_TC)
            {
                Lock<FastMutex> lock(internalStateMutex);
                // Reset the internal button state
                buttonState.ignition_btn         = false;
                buttonState.filling_valve_btn    = false;
                buttonState.venting_valve_btn    = false;
                buttonState.release_pressure_btn = false;
                buttonState.quick_connector_btn  = false;
                buttonState.start_tars_btn       = false;
                buttonState.arm_switch           = false;
            }
        }
    }
    mavlinkWriteToUsart(msg);
}

void Radio::mavlinkWriteToUsart(const mavlink_message_t& msg)
{
    uint8_t temp_buf[MAVLINK_NUM_NON_PAYLOAD_BYTES +
                     MAVLINK_MAX_DIALECT_PAYLOAD_SIZE];
    int len = mavlink_msg_to_send_buffer(temp_buf, &msg);

    auto serial = miosix::DefaultConsole::instance().get();
    serial->writeBlock(temp_buf, len, 0);
}

void Radio::sendInternalState()
{
    mavlink_message_t stateMsg;

    {
        Lock<FastMutex> lock(internalStateMutex);
        mavlink_msg_conrig_state_tc_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &stateMsg, &buttonState);
    }

    mavDriver->enqueueMsg(stateMsg);
}

void Radio::loopReadFromUsart()
{
    mavlink_status_t status;
    mavlink_message_t msg;
    int chan = 0;  // TODO: what is this?
    uint8_t byte;

    while (true)
    {
        auto serial = miosix::DefaultConsole::instance().get();
        int len     = serial->readBlock(&byte, 1, 0);

        if (len && mavlink_parse_char(chan, byte, &msg, &status))
        {
            if (!mavDriver->enqueueMsg(msg))
            {
                mavlink_message_t nack_msg;
                mavlink_nack_tm_t nack_tm;
                nack_tm.recv_msgid = msg.msgid;
                nack_tm.seq_ack    = msg.seq;
                mavlink_msg_nack_tm_encode(Config::Radio::MAV_SYSTEM_ID,
                                           Config::Radio::MAV_COMPONENT_ID,
                                           &nack_msg, &nack_tm);
                mavlinkWriteToUsart(nack_msg);
            }
        }
    }
}

void Radio::setInternalState(mavlink_conrig_state_tc_t state)
{
    Lock<FastMutex> lock(internalStateMutex);
    // The OR operator is introduced to make sure that the receiver
    // understood the command
    buttonState.ignition_btn = state.ignition_btn || buttonState.ignition_btn;
    buttonState.filling_valve_btn =
        state.filling_valve_btn || buttonState.filling_valve_btn;
    buttonState.venting_valve_btn =
        state.venting_valve_btn || buttonState.venting_valve_btn;
    buttonState.release_pressure_btn =
        state.release_pressure_btn || buttonState.release_pressure_btn;
    buttonState.quick_connector_btn =
        state.quick_connector_btn || buttonState.quick_connector_btn;
    buttonState.start_tars_btn =
        state.start_tars_btn || buttonState.start_tars_btn;
    buttonState.arm_switch = state.arm_switch || buttonState.arm_switch;
}

bool Radio::start()
{
    // TODO: constants should be in bps
    using dio0 = Gpio<GPIOB_BASE, 1>;
    using dio1 = Gpio<GPIOD_BASE, 12>;
    using dio3 = Gpio<GPIOD_BASE, 13>;
    using txEn = Gpio<GPIOG_BASE, 2>;
    using rxEn = Gpio<GPIOG_BASE, 3>;
    using cs   = Gpio<GPIOF_BASE, 6>;

    ModuleManager& modules = ModuleManager::getInstance();

    // Config the transceiver
    SX1278Lora::Config config{};
    config.power            = 2;
    config.ocp              = 0;  // Over current protection
    config.coding_rate      = SX1278Lora::Config::Cr::CR_1;
    config.spreading_factor = SX1278Lora::Config::Sf::SF_7;

    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<EbyteFrontend>(txEn::getPin(), rxEn::getPin());

    transceiver =
        new SX1278Lora(modules.get<Buses>()->spi1, cs::getPin(), dio0::getPin(),
                       dio1::getPin(), dio3::getPin(),
                       SPI::ClockDivider::DIV_64, std::move(frontend));

    SX1278Lora::Error error = transceiver->init(config);

    // Config mavDriver
    mavDriver = new MavDriver(transceiver, Config::Radio::MAV_PING_MSG_ID,
                              bind(&Radio::handleMavlinkMessage, this, _1, _2),
                              Config::Radio::MAV_SLEEP_AFTER_SEND);

    // In case of failure the mav driver must be created at least
    if (error != SX1278Lora::Error::NONE)
    {
        return false;
    }

    scheduler->addTask([&]() { sendInternalState(); }, PING_GSE_PERIOD,
                       TaskScheduler::Policy::RECOVER);

    receiverLooper = std::thread([=]() { loopReadFromUsart(); });
    beeperLooper   = std::thread(
        [&]()
        {
            using buzzer = Gpio<GPIOB_BASE, 7>;
            while (true)
            {
                Thread::sleep(100);
                // Doesn't matter the precision, the important thing is
                // the beep, this is because an atomic is used
                if (messageReceived > 5)
                {
                    messageReceived = 0;
                    buzzer::low();
                    Thread::sleep(100);
                    buzzer::high();
                }
            }
        });

    return mavDriver->start();
}

bool Radio::isStarted() { return mavDriver->isStarted(); }

MavlinkStatus Radio::getMavlinkStatus() { return mavDriver->getStatus(); }

Radio::Radio(TaskScheduler* sched) : scheduler(sched)
{
    Lock<FastMutex> lock(internalStateMutex);

    buttonState.ignition_btn         = false;
    buttonState.filling_valve_btn    = false;
    buttonState.venting_valve_btn    = false;
    buttonState.release_pressure_btn = false;
    buttonState.quick_connector_btn  = false;
    buttonState.start_tars_btn       = false;
    buttonState.arm_switch           = false;
}

}  // namespace con_RIG
