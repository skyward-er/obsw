/* Copyright (c) 2018-2019 Skyward Experimental Rocketry
 * Authors: Federico Terraneo, Luca Erbetta
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

/*
 * This is a stub program for the program that will decode the logged data.
 * Fill in the TODO to make it work.
 */

#include <Parafoil/ParafoilTestStatus.h>
#include <Parafoil/Wing/WingAlgorithmData.h>
#include <algorithms/NAS/NASState.h>
#include <common/SystemData.h>
#include <logger/Deserializer.h>
#include <logger/LogTypes.h>
#include <logger/LoggerStats.h>
#include <scheduler/TaskSchedulerData.h>
#include <sensors/BME280/BME280Data.h>
#include <sensors/MPU9250/MPU9250Data.h>
#include <sensors/UBXGPS/UBXGPSData.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <tscpp/stream.h>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "diagnostic/PrintLoggerData.h"
#include "events/EventData.h"

using namespace tscpp;
using namespace Boardcore;
using namespace Parafoil;

void registerTypes(Deserializer& ds)
{
    // Register all Boardcore types
    LogTypes::registerTypes(ds);

    // Custom types

    // Diagnostic
    ds.registerType<TaskStatsResult>();
    ds.registerType<LoggerStats>();
    ds.registerType<LoggingString>();
    ds.registerType<SystemData>();
    ds.registerType<ParafoilTestStatus>();

    // Parafoil data
    ds.registerType<WingAlgorithmData>();

    // Sensors
    ds.registerType<UBXGPSData>();
    ds.registerType<MPU9250Data>();
    ds.registerType<BME280Data>();

    // Nas state
    ds.registerType<NASState>();

    // Others
    ds.registerType<EventData>();
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
    LogTypes::registerTypes(d);
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
