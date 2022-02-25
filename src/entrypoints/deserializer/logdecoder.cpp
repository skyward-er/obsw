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

#include <sys/stat.h>
#include <sys/types.h>
#include <tscpp/stream.h>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace std;
using namespace tscpp;

void showUsage(string cmdName)
{
    std::cerr << "Usage: " << cmdName << " {-a | <log_file_name> | -h}"
              << "Options:\n"
              << "\t-h,--help\t\tShow help message\n"
              << "\t-a,--all Deserialize all logs in the current directory\n"
              << std::endl;
}

bool deserialize(string logName)
{
    cout << "Deserializing " << logName << ".dat...\n";
    Deserializer d(logName);
    registerTypes(d);

    return d.deserialize();
}
bool deserializeAll()
{
    for (int i = 0; i < 100; i++)
    {
        char fn[10];
        char fnext[11];
        sprintf(fn, "log%02d", i);
        sprintf(fnext, "log%02d.dat", i);
        struct stat st;
        if (stat(fnext, &st) != 0)
        {
            // cout << "Skipping " << string(fnext) << "\n ";
            continue;  // File not found
        }
        // File found
        if (!deserialize(string(fn)))
        {
            return false;
        }
    }
    return true;
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        showUsage(string(argv[0]));
        return 1;
    }

    bool success = false;
    string arg   = string(argv[1]);
    if (arg == "-h" || arg == "--help")
    {
        showUsage(string(argv[0]));
        return 0;
    }

    if (arg == "-a" || arg == "--all")
    {
        cout << "Deserializing all logs in the current directory...\n";
        success = deserializeAll();
    }
    else if (arg[0] == '-')
    {
        cerr << "Unknown option\n";
        return 1;
    }
    else
    {
        success = deserialize(arg);
    }

    if (success)
    {
        cout << "Deserialization completed successfully.\n";
    }
    else
    {
        cout << "Deserialization ended with errors.\n";
    }
}
