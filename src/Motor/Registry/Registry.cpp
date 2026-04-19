/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus, Riccardo Sironi
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

using namespace Motor;
using namespace Boardcore;

const char* Motor::configurationIdToName(ConfigurationId id)
{
    switch (id)
    {
        case CONFIG_ID_FULL_THROTTLE_TIME:
            return "FULL_THROTTLE_TIME";
        case CONFIG_ID_LOW_THROTTLE_TIME:
            return "LOW_THROTTLE_TIME";
        case CONFIG_ID_PILOT_FLAME_OX_POSITION:
            return "PILOT_FLAME_OX_POSITION";
        case CONFIG_ID_PILOT_FLAME_FUEL_POSITION:
            return "PILOT_FLAME_FUEL_POSITION";
        case CONFIG_ID_LOW_THROTTLE_OX_POSITION:
            return "LOW_THROTTLE_OX_POSITION";
        case CONFIG_ID_LOW_THROTTLE_FUEL_POSITION:
            return "LOW_THROTTLE_FUEL_POSITION";
        case CONFIG_ID_DEFAULT_OPENING_TIME:
            return "DEFAULT_OPENING_TIME";
        case CONFIG_ID_DEFAULT_MAX_APERTURE:
            return "DEFAULT_MAX_APERTURE";
        case CONFIG_ID_PRZ_FIL_OPENING_TIME:
            return "PRZ_FIL_OPENING_TIME";
        case CONFIG_ID_PRZ_FIL_MAX_APERTURE:
            return "PRZ_FIL_MAX_APERTURE";
        case CONFIG_ID_PRZ_REL_OPENING_TIME:
            return "PRZ_REL_OPENING_TIME";
        case CONFIG_ID_PRZ_REL_MAX_APERTURE:
            return "PRZ_REL_MAX_APERTURE";
        case CONFIG_ID_OX_FIL_OPENING_TIME:
            return "OX_FIL_OPENING_TIME";
        case CONFIG_ID_OX_FIL_MAX_APERTURE:
            return "OX_FIL_MAX_APERTURE";
        case CONFIG_ID_OX_REL_OPENING_TIME:
            return "OX_REL_OPENING_TIME";
        case CONFIG_ID_OX_REL_MAX_APERTURE:
            return "OX_REL_MAX_APERTURE";
        case CONFIG_ID_PRZ_OX_OPENING_TIME:
            return "PRZ_OX_OPENING_TIME";
        case CONFIG_ID_PRZ_OX_MAX_APERTURE:
            return "PRZ_OX_MAX_APERTURE";
        case CONFIG_ID_PRZ_FUEL_OPENING_TIME:
            return "PRZ_FUEL_OPENING_TIME";
        case CONFIG_ID_PRZ_FUEL_MAX_APERTURE:
            return "PRZ_FUEL_MAX_APERTURE";
        case CONFIG_ID_OX_VEN_OPENING_TIME:
            return "OX_VEN_OPENING_TIME";
        case CONFIG_ID_OX_VEN_MAX_APERTURE:
            return "OX_VEN_MAX_APERTURE";
        case CONFIG_ID_FUEL_VEN_OPENING_TIME:
            return "FUEL_VEN_OPENING_TIME";
        case CONFIG_ID_FUEL_VEN_MAX_APERTURE:
            return "FUEL_VEN_MAX_APERTURE";
        case CONFIG_ID_IGN_OX_OPENING_TIME:
            return "IGN_OX_OPENING_TIME";
        case CONFIG_ID_IGN_OX_MAX_APERTURE:
            return "IGN_OX_MAX_APERTURE";
        case CONFIG_ID_IGN_FUEL_OPENING_TIME:
            return "IGN_FUEL_OPENING_TIME";
        case CONFIG_ID_IGN_FUEL_MAX_APERTURE:
            return "IGN_FUEL_MAX_APERTURE";
        case CONFIG_ID_MAIN_OX_OPENING_TIME:
            return "MAIN_OX_OPENING_TIME";
        case CONFIG_ID_MAIN_OX_MAX_APERTURE:
            return "MAIN_OX_MAX_APERTURE";
        case CONFIG_ID_MAIN_FUEL_OPENING_TIME:
            return "MAIN_FUEL_OPENING_TIME";
        case CONFIG_ID_MAIN_FUEL_MAX_APERTURE:
            return "MAIN_FUEL_MAX_APERTURE";

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
