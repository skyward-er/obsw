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
        modules.get<con_RIG::Radio>()->transceiver->handleDioIRQ(
            SX1278::Dio::DIO0);
    }
}

void __attribute__((used)) EXTI12_IRQHandlerImpl()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<con_RIG::Radio>()->transceiver != nullptr)
    {
        modules.get<con_RIG::Radio>()->transceiver->handleDioIRQ(
            SX1278::Dio::DIO1);
    }
}

void __attribute__((used)) EXTI13_IRQHandlerImpl()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<con_RIG::Radio>()->transceiver != nullptr)
    {
        modules.get<con_RIG::Radio>()->transceiver->handleDioIRQ(
            SX1278::Dio::DIO3);
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
                    : 1;  // The beep increments in speed as the state is armed
        }
        case MAVLINK_MSG_ID_ACK_TM:
        {
            int id = mavlink_msg_ack_tm_get_recv_msgid(&msg);
            // we assume this ack is about the last sent message
            if (id == MAVLINK_MSG_ID_CONRIG_STATE_TC)
            {
                ModuleManager::getInstance().get<Buttons>()->resetState();
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

void Radio::sendMessages()
{
    Buttons* buttons   = ModuleManager::getInstance().get<Buttons>();
    ButtonsState state = buttons->getState();

    mavlink_message_t msg;
    mavlink_conrig_state_tc_t tc = {};
    tc.ignition_btn              = state.ignition;
    tc.filling_valve_btn         = state.filling_valve;
    tc.venting_valve_btn         = state.venting_valve;
    tc.release_pressure_btn      = state.release_filling_line_pressure;
    tc.quick_connector_btn       = state.detach_quick_connector;
    tc.start_tars_btn            = state.startup_tars;
    tc.arm_switch                = state.armed;
    mavlink_msg_conrig_state_tc_encode(Config::Radio::MAV_SYSTEM_ID,
                                       Config::Radio::MAV_COMPONENT_ID, &msg,
                                       &tc);
    {
        Lock<FastMutex> lock(mutex);
        for (uint8_t i = 0; i < message_queue_index; i++)
        {
            mavDriver->enqueueMsg(message_queue[i]);
        }
        message_queue_index = 0;

        // The last is the button state message
        mavDriver->enqueueMsg(msg);
    }
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
            if (message_queue_index == MAVLINK_QUEUE_SIZE - 1)
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
            else
            {
                Lock<FastMutex> lock(mutex);
                message_queue[message_queue_index] = msg;
                message_queue_index += 1;
            }
        }
    }
}

bool Radio::start()
{
    // TODO: constants should be in bps
    using dio0 = Gpio<GPIOB_BASE, 1>;
    using dio1 = Gpio<GPIOD_BASE, 12>;
    using dio3 = Gpio<GPIOD_BASE, 13>;

    ModuleManager& modules = ModuleManager::getInstance();

    SPIBusConfig spiConfig{};
    spiConfig.clockDivider = SPI::ClockDivider::DIV_16;
    spiConfig.mode         = SPI::Mode::MODE_0;
    spiConfig.bitOrder     = SPI::BitOrder::MSB_FIRST;

    // Config the transceiver
    SX1278Lora::Config config{};
    config.pa_boost         = true;
    config.power            = 2;
    config.ocp              = 0;  // Over current protection
    config.coding_rate      = SX1278Lora::Config::Cr::CR_1;
    config.spreading_factor = SX1278Lora::Config::Sf::SF_7;

    transceiver = new EbyteLora(
        SPISlave(modules.get<Buses>()->spi1, Gpio<GPIOF_BASE, 6>::getPin(),
                 spiConfig),
        Gpio<GPIOG_BASE, 2>::getPin(), Gpio<GPIOG_BASE, 3>::getPin());

    enableExternalInterrupt(dio0::getPin().getPort(),
                            dio0::getPin().getNumber(),
                            InterruptTrigger::RISING_EDGE);
    enableExternalInterrupt(dio1::getPin().getPort(),
                            dio1::getPin().getNumber(),
                            InterruptTrigger::RISING_EDGE);
    enableExternalInterrupt(dio3::getPin().getPort(),
                            dio3::getPin().getNumber(),
                            InterruptTrigger::RISING_EDGE);

    SX1278Lora::Error error = transceiver->init(config);

    // Config mavDriver
    mavDriver = new MavDriver(transceiver,
                              bind(&Radio::handleMavlinkMessage, this, _1, _2),
                              0, MAV_OUT_BUFFER_MAX_AGE);

    // In case of failure the mav driver must be created at least
    if (error != SX1278Lora::Error::NONE)
    {
        return false;
    }

    scheduler->addTask([&]() { sendMessages(); }, PING_GSE_PERIOD,
                       PING_GSE_TASK_ID, TaskScheduler::Policy::RECOVER);

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

Radio::Radio(TaskScheduler* sched) : scheduler(sched) {}

}  // namespace con_RIG
