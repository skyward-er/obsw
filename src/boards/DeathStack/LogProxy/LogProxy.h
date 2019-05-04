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

#include "FlightStats.h"
#include "Singleton.h"
#include "logger/Logger.h"

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

    ~LoggerProxy() {}

    /* Generic log function, to be implemented for each loggable struct */
    template <typename T>
    inline LogResult log(const T& t)
    {
        return logger.log(t);
    }

    /**
     * Blocking call. May take a long time.
     *
     * Call this function to start the logger.
     * When this function returns, the logger is started, and subsequent calls
     * to log will actually log the data.
     *
     * \throws runtime_error if the log could not be opened
     * \return log number
     */
    int start() { return logger.start(); }

    /**
     * Blocking call. May take a very long time (seconds).
     *
     * Call this function to stop the logger.
     * When this function returns, all log buffers have been flushed to disk,
     * and it is safe to power down the board without losing log data or
     * corrupting the filesystem.
     */
    void stop() { logger.stop(); }

private:
    Logger& logger;  // SD logger
    FlightStats flight_stats{};
};

}  // namespace DeathStackBoard