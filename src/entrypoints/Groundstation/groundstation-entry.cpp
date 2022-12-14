/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#include <common/Mavlink.h>
#include <common/SX1278Config.h>
#include <drivers/interrupt/external_interrupts.h>
#include <drivers/timer/TimestampTimer.h>
#include <drivers/usart/USART.h>
#include <filesystem/console/console_device.h>
#include <interfaces-impl/hwmapping.h>
#include <radio/SX1278/SX1278.h>
#include <utils/MovingAverage.h>

#include <thread>

#include "GUI/GUI.h"

#define USE_RA01_PC13

using namespace miosix;
using namespace Boardcore;
using namespace Common;

SX1278 *sx1278 = nullptr;
USART *usart   = nullptr;
GUI *gui       = nullptr;

Mutex usart_mutex;

#ifdef USE_RA01_PC13
void __attribute__((used)) EXTI6_IRQHandlerImpl()
#else
void __attribute__((used)) EXTI3_IRQHandlerImpl()
#endif
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

constexpr size_t STATS_PERIOD = 250;

struct Stats
{
    MovingAverage<uint64_t, 10> tx;
    MovingAverage<uint64_t, 10> rx;

    uint64_t cur_tx = 0;
    uint64_t cur_rx = 0;
    int sent_count  = 0;
    int recv_count  = 0;
    float rssi      = 0.0f;
    float fei       = 0.0f;

    uint64_t txBitrate() { return (tx.getAverage() * 1000) / STATS_PERIOD; }
    uint64_t rxBitrate() { return (rx.getAverage() * 1000) / STATS_PERIOD; }

    mavlink_receiver_tm_t toMavlink()
    {
        mavlink_receiver_tm_t tm = {};
        tm.rssi                  = rssi;
        tm.fei                   = fei;
        tm.tx_bitrate            = txBitrate();
        tm.rx_bitrate            = rxBitrate();

        return tm;
    }
} stats;

void usartWriteMavlink(const mavlink_message_t &msg)
{
    Lock<Mutex> lock(usart_mutex);

    uint8_t temp_buf[MAVLINK_NUM_NON_PAYLOAD_BYTES +
                     MAVLINK_MAX_DIALECT_PAYLOAD_SIZE];
    int len = mavlink_msg_to_send_buffer(temp_buf, &msg);

    usart->write(temp_buf, len);
}

void sendStats()
{
    mavlink_receiver_tm_t tm = stats.toMavlink();
    tm.timestamp             = TimestampTimer::getTimestamp();

    mavlink_message_t msg;
    mavlink_msg_receiver_tm_encode(0, 0, &msg, &tm);

    usartWriteMavlink(msg);
}

void recvLoop()
{
    uint8_t buf[63];
    mavlink_message_t msg;
    mavlink_status_t status;

    while (true)
    {
        int len    = sx1278->receive(buf, sizeof(msg));
        stats.rssi = sx1278->getLastRxRssi();
        stats.fei  = sx1278->getLastRxFei();
        stats.recv_count++;
        stats.cur_rx += len;

        for (int i = 0; i < len; i++)
        {
            uint8_t result =
                mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status);

            if (result == 1)
                usartWriteMavlink(msg);
        }

        // usart->write(msg, len);
    }
}

void sendLoop()
{
    uint8_t msg[63];

    while (true)
    {
        int len = usart->read(msg, sizeof(msg));

        ledOn();
        sx1278->send(msg, len);
        stats.sent_count++;
        stats.cur_tx += len;
        ledOff();
    }
}

void initBoard()
{
    // Enable dio0 interrupt
#ifdef USE_RA01_PC13
    enableExternalInterrupt(GPIOF_BASE, 6, InterruptTrigger::RISING_EDGE);
#else
    enableExternalInterrupt(GPIOE_BASE, 3, InterruptTrigger::RISING_EDGE);
#endif
}

void initGUI()
{
    // TODO: This should be in bsp
    using GpioUserBtn = Gpio<GPIOA_BASE, 0>;
    GpioUserBtn::mode(Mode::INPUT_PULL_DOWN);

    gui = new GUI();

    ButtonHandler::getInstance().registerButtonCallback(
        GpioUserBtn::getPin(),
        [](auto event) { gui->screen_manager.onButtonEvent(event); });
}

void initUART()
{
    usart = new USART(USART1, USARTInterface::Baudrate::B115200);
    usart->init();
}

int main()
{
    initBoard();
    initGUI();

    SPIBus bus(SPI4);
#ifdef USE_RA01_PC13
    GpioPin cs = peripherals::ra01::pc13::cs::getPin();
#else
    GpioPin cs = peripherals::ra01::pe4::cs::getPin();
#endif

    sx1278 = new SX1278(bus, cs);

    printf("[sx1278] Configuring sx1278...\n");
    SX1278::Error err;
    if ((err = sx1278->init(SX1278_CONFIG)) != SX1278::Error::NONE)
    {
        gui->stats_screen.updateError(err);
        while (true)
        {
            printf("[SX1278] Initialization failed\n");
            Thread::sleep(1000);
        }
    }
    printf("[sx1278] Initialization complete!\n");

    for (int i = 0; i < 5; i++)
    {
        ledOn();
        Thread::sleep(100);
        ledOff();
        Thread::sleep(100);
    }

    gui->stats_screen.updateReady();

    initUART();

    std::thread recv([]() { recvLoop(); });
    std::thread send([]() { sendLoop(); });

    while (true)
    {
        long long start = miosix::getTick();

        stats.tx.push(stats.cur_tx);
        stats.rx.push(stats.cur_rx);
        stats.cur_rx = 0;
        stats.cur_tx = 0;

        StatsScreen::Data data = {stats.txBitrate() * 8,
                                  stats.rxBitrate() * 8,
                                  stats.sent_count,
                                  stats.recv_count,
                                  stats.rssi,
                                  stats.fei};

        gui->stats_screen.updateStats(data);
        sendStats();

        Thread::sleepUntil(start + STATS_PERIOD);
    }
}
