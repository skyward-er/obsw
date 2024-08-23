/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <Payload/PinHandler/PinData.h>
#include <Payload/Sensors/SensorData.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManagerData.h>
#include <Payload/StateMachines/NASController/NASControllerData.h>
#include <Payload/StateMachines/WingController/WingControllerData.h>
#include <logger/Deserializer.h>
#include <logger/LogTypes.h>
#include <logger/Logger.h>
#include <tscpp/stream.h>

#include <filesystem>
#include <format>
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
using namespace Payload;

void registerTypes(Deserializer& ds)
{
    // Register all Boardcore types
    LogTypes::registerTypes(ds);

    // Additional types defined in the payload software
    ds.registerType<FlightModeManagerStatus>();
    ds.registerType<NASControllerStatus>();
    ds.registerType<WingControllerStatus>();
    ds.registerType<StaticPressureData>();
    ds.registerType<DynamicPressureData>();
    ds.registerType<SensorsCalibrationParameter>();
    ds.registerType<PinChangeData>();
}

// cppcheck-suppress passedByValue
void printUsage(std::string_view cmdName)
{
    std::cerr << "Usage: " << cmdName << " [OPTION]... [FILE]...\n"
              << "Payload log files decoder.\n\n"
              << "With no FILE, deserialize all log files in the current "
                 "directory.\n\n"
              << "Options:\n"
              << "  -h, --help\t\tDisplay this help and exit\n";
}

bool deserialize(const std::filesystem::path& file)
{
    std::cout << "Deserializing " << file << "\n";

    Deserializer d(file);
    registerTypes(d);

    auto start  = steady_clock::now();
    bool result = d.deserialize();
    auto end    = steady_clock::now();

    if (result)
    {
        std::cout << "Successfully deserialized " << file << " in "
                  << duration_cast<milliseconds>(end - start) << "\n";
    }
    else
    {
        std::cerr << "Failed to deserialize " << file << "\n";
    }

    return result;
}

bool deserializeDirectory(const std::filesystem::path& directory)
{
    std::cout << "Deserializing all log files in " << directory << "\n";

    // Deserialize log files in the directory they are in
    std::filesystem::current_path(directory);

    bool success = true;
    for (int i = 0; i < Logger::getMaxFilenameNumber(); i++)
    {
        auto file = std::format("log{:02d}.dat", i);
        if (!std::filesystem::is_regular_file(file))
        {
            continue;
        }

        success &= deserialize(file);
    }

    return success;
}

int main(int argc, char* argv[])
{
    auto exeName = argv[0];

    // A list of files and directories to deserialize
    std::vector<std::filesystem::path> files;

    // Parse arguments
    for (int i = 1; i < argc; i++)
    {
        auto arg = std::string_view(argv[i]);

        // Help option
        if (arg == "-h" || arg == "--help")
        {
            printUsage(exeName);
            return 0;
        }

        // Invalid option
        if (arg.starts_with("-"))
        {
            std::cerr << exeName << ": invalid option "
                      << std::quoted(arg, '\'') << "\n"
                      << "Try '" << exeName
                      << " --help' for more information.\n";
            return 1;
        }

        // File or directory argument
        auto file = std::filesystem::path(arg);
        if (std::filesystem::exists(arg))
        {
            files.push_back(file);
        }
        else
        {
            std::cerr << "File " << file << " does not exist\n";
            return 1;
        }
    }

    // If no files are provided, deserialize all log files in the current dir
    if (files.empty())
    {
        files.push_back(std::filesystem::current_path());
    }

    bool success = true;
    // Deserialize all files
    for (const auto& file : files)
    {
        if (std::filesystem::is_directory(file))
        {
            success &= deserializeDirectory(file);
        }
        else
        {
            success &= deserialize(file);
        }
    }

    if (success)
    {
        std::cout << "Done\n";
        return 0;
    }
    else
    {
        std::cerr << "Failed to deserialize one or more files\n";
        return 1;
    }
}
