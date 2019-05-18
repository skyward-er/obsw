/***************************************************************************
 *   Copyright (C) 2018 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/

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

// TODO: add here include files of serialized classes

#include "LogTypes.h"

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
            //cout << "Skipping " << string(fnext) << "\n ";
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
