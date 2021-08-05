/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <LoggerService/LoggerService.h>
#include <Main/Radio.h>
#include <TelemetriesTelecommands/TCHandler.h>
#include <TelemetriesTelecommands/TMTCController.h>
#include <TelemetriesTelecommands/TmRepository.h>
#include <drivers/Xbee/APIFramesLog.h>
#include <drivers/Xbee/ATCommands.h>
#include <drivers/interrupt/external_interrupts.h>
#include <interfaces-impl/hwmapping.h>

#include <functional>

// using std::function;
using std::bind;
using namespace std::placeholders;

// Xbee ATTN interrupt
void __attribute__((used)) EXTI10_IRQHandlerImpl()
{
    using namespace DeathStackBoard;

    if (DeathStack::getInstance()->radio->xbee != nullptr)
    {
        DeathStack::getInstance()->radio->xbee->handleATTNInterrupt();
    }
}

namespace DeathStackBoard
{

Radio::Radio(SPIBusInterface& xbee_bus) : xbee_bus(xbee_bus)
{
    SPIBusConfig xbee_cfg{};

    xbee_cfg.clock_div = SPIClockDivider::DIV16;

    xbee = new Xbee::Xbee(xbee_bus, xbee_cfg, miosix::xbee::cs::getPin(),
                          miosix::xbee::attn::getPin(),
                          miosix::xbee::reset::getPin());
    xbee->setOnFrameReceivedListener(
        bind(&Radio::onXbeeFrameReceived, this, _1));

    Xbee::setDataRate(*xbee, XBEE_80KBPS_DATA_RATE, 5000);

    mav_driver = new MavDriver(xbee, handleMavlinkMessage, 0,
                               1000);  // TODO: Use settings

    tmtc_manager = new TMTCController();

    tm_repo = TmRepository::getInstance();

    enableExternalInterrupt(GPIOF_BASE, 10, InterruptTrigger::FALLING_EDGE);
}

Radio::~Radio()
{
    tmtc_manager->stop();
    mav_driver->stop();

    delete tmtc_manager;
    delete mav_driver;
    delete xbee;
}

bool Radio::start() { return mav_driver->start() && tmtc_manager->start(); }

void Radio::onXbeeFrameReceived(Xbee::APIFrame& frame)
{
    LoggerService& logger = *LoggerService::getInstance();

    using namespace Xbee;
    bool logged = false;
    switch (frame.frame_type)
    {
        case FTYPE_AT_COMMAND:
        {
            ATCommandFrameLog dest;
            logged = ATCommandFrameLog::toFrameType(frame, &dest);
            if (logged)
            {
                logger.log(dest);
            }
            break;
        }
        case FTYPE_AT_COMMAND_RESPONSE:
        {
            ATCommandResponseFrameLog dest;
            logged = ATCommandResponseFrameLog::toFrameType(frame, &dest);
            if (logged)
            {
                logger.log(dest);
            }
            break;
        }
        case FTYPE_MODEM_STATUS:
        {
            ModemStatusFrameLog dest;
            logged = ModemStatusFrameLog::toFrameType(frame, &dest);
            if (logged)
            {
                logger.log(dest);
            }
            break;
        }
        case FTYPE_TX_REQUEST:
        {
            TXRequestFrameLog dest;
            logged = TXRequestFrameLog::toFrameType(frame, &dest);
            if (logged)
            {
                logger.log(dest);
            }
            break;
        }
        case FTYPE_TX_STATUS:
        {
            TXStatusFrameLog dest;
            logged = TXStatusFrameLog::toFrameType(frame, &dest);
            if (logged)
            {
                logger.log(dest);
            }
            break;
        }
        case FTYPE_RX_PACKET_FRAME:
        {
            RXPacketFrameLog dest;
            logged = RXPacketFrameLog::toFrameType(frame, &dest);
            if (logged)
            {
                logger.log(dest);
            }
            break;
        }
    }

    if (!logged)
    {
        APIFrameLog api;
        APIFrameLog::fromAPIFrame(frame, &api);
        logger.log(api);
    }
}

}  // namespace DeathStackBoard