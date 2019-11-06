/* Copyright (c) 2018 Skyward Experimental Rocketry
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

/*
 ******************************************************************************
 *                  THIS FILE IS AUTOGENERATED. DO NOT EDIT.                  *
 ******************************************************************************
 */

// Generated from:  https://docs.google.com/spreadsheets/d/184kR2OAD7yWV0fYJdiGUDmHmy5_prY3nr-XgNA0Uge0
// Autogen date:    2019-11-06 14:46:13.574146

#pragma once

#include <stdint.h>
#include <string>
#include <vector>

using std::string;

namespace DeathStackBoard
{
/**
 * Definition of various event topics to use in the EventBroker
 */
enum Topics : uint8_t
{
    TOPIC_ADA,
    TOPIC_CAN,
    TOPIC_DEPLOYMENT,
    TOPIC_FLIGHT_EVENTS,
    TOPIC_FMM,
    TOPIC_IGNITION,
    TOPIC_STATS,
    TOPIC_TC,
    TOPIC_TMTC
};

const std::vector<uint8_t> TOPIC_LIST {TOPIC_ADA, TOPIC_CAN, TOPIC_DEPLOYMENT, TOPIC_FLIGHT_EVENTS, TOPIC_FMM, TOPIC_IGNITION, TOPIC_STATS, TOPIC_TC, TOPIC_TMTC};

/**
 * @brief Returns the name of the provided event
 *
 * @param event
 * @return string
 */
string getTopicString(uint8_t topic);

}  // namespace DeathStackBoard

