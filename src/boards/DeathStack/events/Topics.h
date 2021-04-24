/* Copyright (c) 2018-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alvise de' Faveri Tron
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

// Autogen date: 2021-04-17 23:09:11.522877

#pragma once

#include <cstdint>
#include <string>
#include <vector>

using std::string;

/**
 * Definition of various event topics to use in the EventBroker
 */
enum Topics : uint8_t
{
    TOPIC_FLIGHT_EVENTS,
    TOPIC_FMM,
    TOPIC_SM,
    TOPIC_TC,
    TOPIC_DPL,
    TOPIC_ABK,
    TOPIC_NAS,
    TOPIC_TMTC,
    TOPIC_STATS,
    TOPIC_ADA,
};

const std::vector<uint8_t> TOPIC_LIST{
    TOPIC_FLIGHT_EVENTS,
    TOPIC_FMM,
    TOPIC_SM,
    TOPIC_TC,
    TOPIC_DPL,
    TOPIC_ABK,
    TOPIC_NAS,
    TOPIC_TMTC,
    TOPIC_STATS,
    TOPIC_ADA,
};

/**
 * @brief Returns the name of the provided event
 *
 * @param event
 * @return string
 */
string getTopicString(uint8_t topic);