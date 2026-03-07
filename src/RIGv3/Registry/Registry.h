/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus
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

#pragma once

#include <utils/DependencyManager/DependencyManager.h>
#include <utils/Registry/RegistryFrontend.h>

namespace RIGv3
{

enum ConfigurationKeys
{

    // Ignition parameters
    CONFIG_ID_IGNITION_TIME,

    // Default valve parameters
    CONFIG_ID_DEFAULT_OPENING_TIME,
    CONFIG_ID_DEFAULT_MAX_APERTURE,

    CONFIG_ID_COOLING_DELAY,  // Delay of cooling procedure after end of firing

    // TARS3 parameters
    CONFIG_ID_TARS3_MASS_TARGET,
    CONFIG_ID_TARS3_PRESSURE_TARGET,
};

const char* configurationIdToName(Boardcore::ConfigurationId id);

class Registry : public Boardcore::Injectable,
                 public Boardcore::RegistryFrontend
{
public:
    Registry();

    [[nodiscard]] bool start();
};

}  // namespace RIGv3
