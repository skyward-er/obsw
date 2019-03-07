/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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

#include "logger/Logger.h"
#include "Singleton.h"

namespace DeathStackBoard 
{

/**
 * @brief This class is interposed between the OBSW and the Logger driver. 
 * Status repository updating is done here: everytime a component 
 * logs its status, the corresponding tm structs are updated before logging
 * on SD card.
 */
class LoggerProxy : public Singleton<LoggerProxy>
{
    friend class Singleton<LoggerProxy>;

public:

    LoggerProxy() : logger(Logger::instance()) {}

    /* Generic log function, to be implemented for each loggable struct */
    template <typename T>
    inline LogResult log(const T& t)
    {
        return logger.log(t);
    }


private:
    Logger& logger; // SD logger
};

}