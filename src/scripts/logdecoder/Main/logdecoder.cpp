/* Copyright (c) 2018-2022 Skyward Experimental Rocketry
 * Author: Terrane Federico
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

// #include <Main/Sensors/SensorsData.h>
#include <Main/CanHandler/CanHandlerData.h>
#include <Main/PinHandler/PinData.h>
#include <Main/Sensors/RotatedIMU/RotatedIMUData.h>
#include <Main/Sensors/SensorsData.h>
#include <Main/StateMachines/ABKController/ABKControllerData.h>
#include <Main/StateMachines/ADAController/ADAControllerData.h>
#include <Main/StateMachines/Deployment/DeploymentData.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManagerData.h>
#include <Main/StateMachines/MEAController/MEAControllerData.h>
#include <Main/StateMachines/NASController/NASControllerData.h>
#include <algorithms/MEA/MEAData.h>
#include <hardware_in_the_loop/HIL_sensors/HILSensorsData.h>
#include <logger/Deserializer.h>
#include <logger/LogTypes.h>
#include <tscpp/stream.h>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

/**
 * @brief Binary log files decoder.
 *
 * This program is to compile for you computer and decodes binary log files
 * through the tscpp library.
 *
 * In LogTypes.h there should be included all the classes you want to
 * deserialize.
 */

using namespace tscpp;
using namespace Boardcore;
using namespace Main;

void registerTypes(Deserializer& ds)
{
    // Register all Boardcore types
    LogTypes::registerTypes(ds);

    // Custom types
    ds.registerType<FlightModeManagerStatus>();
    ds.registerType<NASControllerStatus>();
    ds.registerType<MEAControllerStatus>();
    ds.registerType<MEAState>();
    ds.registerType<ADAControllerStatus>();
    ds.registerType<ABKControllerStatus>();
    ds.registerType<DeploymentStatus>();
    ds.registerType<LPS28DFW_1Data>();
    ds.registerType<LPS28DFW_2Data>();
    ds.registerType<HSCMRNN015PA_1Data>();
    ds.registerType<HSCMRNN015PA_2Data>();
    ds.registerType<RotatedIMUData>();
    ds.registerType<CanPressureSensor>();
    ds.registerType<CanTemperatureSensor>();
    ds.registerType<CanCurrentSensor>();
    ds.registerType<CanVoltageSensor>();
    ds.registerType<SensorsCalibrationParameter>();
    ds.registerType<PinChangeData>();
    ds.registerType<HILAccelerometerData>();
    ds.registerType<HILBarometerData>();
    ds.registerType<HILGpsData>();
    ds.registerType<HILGyroscopeData>();
    ds.registerType<HILImuData>();
    ds.registerType<HILMagnetometerData>();
    ds.registerType<HILPitotData>();
    ds.registerType<HILTempData>();
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
