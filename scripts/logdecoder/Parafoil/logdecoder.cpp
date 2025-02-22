/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Davide Basso
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

#include <Parafoil/PinHandler/PinData.h>
#include <Parafoil/Sensors/SensorData.h>
#include <Parafoil/StateMachines/FlightModeManager/FlightModeManagerData.h>
#include <Parafoil/StateMachines/NASController/NASControllerData.h>
#include <Parafoil/StateMachines/WingController/WingControllerData.h>
#include <Parafoil/WindEstimation/WindEstimationData.h>
#include <Parafoil/Wing/WingAlgorithmData.h>
#include <Parafoil/Wing/WingTargetPositionData.h>
#include <fmt/format.h>
#include <logger/Deserializer.h>
#include <logger/LogTypes.h>
#include <logger/Logger.h>
#include <radio/Xbee/APIFramesLog.h>
#include <tscpp/stream.h>

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string_view>

/**
 * @brief Binary log files decoder.
 *
 * This program is to compile for you computer and decodes binary log files
 * through the tscpp library.
 *
 * In LogTypes.h there should be included all the classes you want to
 * deserialize.
 */

using namespace std::chrono;
using namespace tscpp;
using namespace Boardcore;
using namespace Parafoil;

void registerTypes(Deserializer& ds)
{
    // Register all Boardcore types
    LogTypes::registerTypes(ds);

    // Additional types defined in the parafoil software
    ds.registerType<FlightModeManagerStatus>();
    ds.registerType<NASControllerStatus>();
    ds.registerType<WingControllerStatus>();
    ds.registerType<SensorCalibrationData>();
    ds.registerType<PinChangeData>();
    ds.registerType<WingControllerAlgorithmData>();
    ds.registerType<WingAlgorithmData>();
    ds.registerType<WingTargetPositionData>();
    ds.registerType<EarlyManeuversActiveTargetData>();
    ds.registerType<Xbee::ATCommandFrameLog>();
    ds.registerType<Xbee::ATCommandResponseFrameLog>();
    ds.registerType<Xbee::ModemStatusFrameLog>();
    ds.registerType<Xbee::TXRequestFrameLog>();
    ds.registerType<Xbee::TXStatusFrameLog>();
    ds.registerType<Xbee::RXPacketFrameLog>();
    ds.registerType<MavlinkStatus>();
    ds.registerType<WingTargetPositionData>();
    ds.registerType<WingControllerAlgorithmData>();
    ds.registerType<WingControllerStatus>();
    ds.registerType<WindEstimationData>();
}

void showUsage(const string& cmdName)
{
    std::cerr << "Usage: " << cmdName << " {-a | <log_file_name> | -h}"
              << "Options:\n"
              << "\t-h,--help\t\tShow help message\n"
              << "\t-a,--all Deserialize all logs in the current directory\n"
              << std::endl;
}

bool deserialize(string logName)
{
    std::cout << "Deserializing " << logName << "...\n";
    Deserializer d(logName);
    registerTypes(d);

    return d.deserialize();
}

bool deserializeAll()
{
    for (int i = 0; i < 100; i++)
    {
        char nextName[11];
        sprintf(nextName, "log%02d.dat", i);
        struct stat st;
        if (stat(nextName, &st) != 0)
            continue;  // File not found
        // File found
        if (!deserialize(string(nextName)))
            return false;
    }
    return true;
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        showUsage(string(argv[0]));
        return 1;  // Error
    }

    bool success = false;
    string arg1  = string(argv[1]);

    // Help message
    if (arg1 == "-h" || arg1 == "--help")
    {
        showUsage(string(argv[0]));
        return 0;
    }

    // File deserialization
    if (arg1 == "-a" || arg1 == "--all")
        success = deserializeAll();
    else
        success = deserialize(arg1);

    // End
    if (success)
        std::cout << "Deserialization completed successfully\n";
    else
        std::cout << "Deserialization ended with errors\n";
    return 0;
}

