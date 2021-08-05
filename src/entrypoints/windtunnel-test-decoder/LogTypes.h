/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#pragma once

#include <fstream>
#include <iostream>

#include "AirBrakesController/AirBrakesData.h"
#include "AirBrakesController/WindData.h"
#include "DeathStackStatus.h"
#include "drivers/Xbee/APIFramesLog.h"
#include "drivers/Xbee/XbeeStatus.h"
#include "drivers/adc/ADS1118/ADS1118Data.h"
#include "drivers/mavlink/MavlinkStatus.h"
#include "events/EventData.h"
#include "logger/Deserializer.h"
#include "logger/LogStats.h"
#include "sensors/MS580301BA07/MS580301BA07Data.h"
#include "sensors/analog/pressure/MPXHZ6130A/MPXHZ6130AData.h"
#include "sensors/analog/pressure/honeywell/SSCDANN030PAAData.h"
#include "sensors/analog/pressure/honeywell/SSCDRRN015PDAData.h"

// Serialized classes
using std::ofstream;

using namespace DeathStackBoard;

template <typename T>
void print(T& t, ostream& os)
{
    t.print(os);
}

template <typename T>
void registerType(Deserializer& ds)
{
    ds.registerType<T>(print<T>, T::header());
}

void registerTypes(Deserializer& ds)
{
    registerType<LogStats>(ds);
    registerType<EventData>(ds);

    registerType<MS5803Data>(ds);
    registerType<ADS1118Data>(ds);
    registerType<SSCDRRN015PDAData>(ds);
    registerType<SSCDANN030PAAData>(ds);
    registerType<MPXHZ6130AData>(ds);

    registerType<Xbee::APIFrameLog>(ds);
    registerType<Xbee::ATCommandFrameLog>(ds);
    registerType<Xbee::ATCommandResponseFrameLog>(ds);
    registerType<Xbee::ModemStatusFrameLog>(ds);
    registerType<Xbee::TXRequestFrameLog>(ds);
    registerType<Xbee::TXStatusFrameLog>(ds);
    registerType<Xbee::RXPacketFrameLog>(ds);
    registerType<Xbee::XbeeStatus>(ds);
    registerType<Xbee::XbeeStatus>(ds);

    registerType<Xbee::XbeeStatus>(ds);
    registerType<DeathStackBoard::DeathStackStatus>(ds);
    registerType<MavlinkStatus>(ds);
    registerType<AirBrakesData>(ds);
    registerType<WindData>(ds);
}
