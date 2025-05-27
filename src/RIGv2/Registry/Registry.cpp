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

#include "Registry.h"

#include <utils/Registry/Backend/FileBackend.h>

#include <fstream>

using namespace RIGv2;
using namespace Boardcore;

const char* RIGv2::configurationIdToName(ConfigurationId id)
{
    switch (id)
    {
        case CONFIG_ID_OX_FIL_OPENING_TIME:
            return "OX_FIL_OPENING_TIME";
        case CONFIG_ID_OX_FIL_MAX_APERTURE:
            return "OX_FIL_MAX_APERTURE";
        case CONFIG_ID_OX_REL_OPENING_TIME:
            return "OX_REL_OPENING_TIME";
        case CONFIG_ID_OX_REL_MAX_APERTURE:
            return "OX_REL_MAX_APERTURE";
        case CONFIG_ID_OX_DET_OPENING_TIME:
            return "OX_DET_OPENING_TIME";
        case CONFIG_ID_OX_DET_MAX_APERTURE:
            return "OX_DET_MAX_APERTURE";
        case CONFIG_ID_OX_VEN_OPENING_TIME:
            return "OX_VEN_OPENING_TIME";
        case CONFIG_ID_OX_VEN_MAX_APERTURE:
            return "OX_VEN_MAX_APERTURE";
        case CONFIG_ID_N2_FIL_OPENING_TIME:
            return "N2_FIL_OPENING_TIME";
        case CONFIG_ID_N2_FIL_MAX_APERTURE:
            return "N2_FIL_MAX_APERTURE";
        case CONFIG_ID_N2_REL_OPENING_TIME:
            return "N2_REL_OPENING_TIME";
        case CONFIG_ID_N2_REL_MAX_APERTURE:
            return "N2_REL_MAX_APERTURE";
        case CONFIG_ID_N2_DET_OPENING_TIME:
            return "N2_DET_OPENING_TIME";
        case CONFIG_ID_N2_DET_MAX_APERTURE:
            return "N2_DET_MAX_APERTURE";
        case CONFIG_ID_N2_QUE_OPENING_TIME:
            return "N2_QUE_OPENING_TIME";
        case CONFIG_ID_N2_QUE_MAX_APERTURE:
            return "N2_QUE_MAX_APERTURE";
        case CONFIG_ID_MAIN_OPENING_TIME:
            return "MAIN_OPENING_TIME";
        case CONFIG_ID_MAIN_MAX_APERTURE:
            return "MAIN_MAX_APERTURE";
        case CONFIG_ID_NITR_OPENING_TIME:
            return "NITR_OPENING_TIME";
        case CONFIG_ID_NITR_MAX_APERTURE:
            return "NITR_MAX_APERTURE";
        case CONFIG_ID_IGNITION_TIME:
            return "IGNITION_TIME";
        case CONFIG_ID_DEFAULT_OPENING_TIME:
            return "DEFAULT_OPENING_TIME";
        case CONFIG_ID_DEFAULT_MAX_APERTURE:
            return "DEFAULT_MAX_APERTURE";
        case CONFIG_ID_CHAMBER_TIME:
            return "CHAMBER_TIME";
        case CONFIG_ID_CHAMBER_DELAY:
            return "CHAMBER_DELAY";
        case CONFIG_ID_TARS3_MASS_TARGET:
            return "TARS3_MASS_TARGET";
        case CONFIG_ID_TARS3_PRESSURE_TARGET:
            return "TARS3_PRESSURE_TARGET";

        default:
            return "<invalid>";
    }
}

Registry::Registry()
    : RegistryFrontend(std::make_unique<FileBackend>("/sd/registryStore.bin"))
{
}

bool Registry::start()
{
    return RegistryFrontend::start() == RegistryError::OK;
}
