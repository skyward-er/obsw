/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

namespace RIGv2
{

enum ConfigurationKeys
{
    // Ox (N2O)
    CONFIG_ID_OX_FIL_OPENING_TIME,
    CONFIG_ID_OX_FIL_MAX_APERTURE,

    CONFIG_ID_OX_REL_OPENING_TIME,
    CONFIG_ID_OX_REL_MAX_APERTURE,

    CONFIG_ID_OX_DET_OPENING_TIME,
    CONFIG_ID_OX_DET_MAX_APERTURE,

    CONFIG_ID_OX_VEN_OPENING_TIME,
    CONFIG_ID_OX_VEN_MAX_APERTURE,

    // Pressurizer (N2)
    CONFIG_ID_PRZ_FIL_OPENING_TIME,
    CONFIG_ID_PRZ_FIL_MAX_APERTURE,

    CONFIG_ID_PRZ_REL_OPENING_TIME,
    CONFIG_ID_PRZ_REL_MAX_APERTURE,

    CONFIG_ID_PRZ_DET_OPENING_TIME,
    CONFIG_ID_PRZ_DET_MAX_APERTURE,

    CONFIG_ID_PRZ_QUE_OPENING_TIME,
    CONFIG_ID_PRZ_QUE_MAX_APERTURE,

    // Fuel
    CONFIG_ID_FUEL_VEN_OPENING_TIME,
    CONFIG_ID_FUEL_VEN_MAX_APERTURE,

    // Main
    CONFIG_ID_MAIN_FUEL_OPENING_TIME,
    CONFIG_ID_MAIN_FUEL_MAX_APERTURE,

    CONFIG_ID_MAIN_OX_OPENING_TIME,
    CONFIG_ID_MAIN_OX_MAX_APERTURE,

    // Tank pressurizer
    CONFIG_ID_PRZ_FUEL_OPENING_TIME,
    CONFIG_ID_PRZ_FUEL_MAX_APERTURE,

    CONFIG_ID_PRZ_OX_OPENING_TIME,
    CONFIG_ID_PRZ_OX_MAX_APERTURE,

    // Ignition parameters
    CONFIG_ID_IGNITION_TIME,

    // Default valve parameters
    CONFIG_ID_DEFAULT_OPENING_TIME,
    CONFIG_ID_DEFAULT_MAX_APERTURE,

    // Chamber valve parameters
    CONFIG_ID_CHAMBER_TIME,   // Time the chamber valve stays open
    CONFIG_ID_CHAMBER_DELAY,  // Delay of opening the chamber valve after
                              // opening the main valve

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

}  // namespace RIGv2
