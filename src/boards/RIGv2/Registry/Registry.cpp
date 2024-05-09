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

#include <fstream>

using namespace RIGv2;
using namespace Boardcore;

const char* RIGv2::configurationIdToName(ConfigurationId id)
{
    switch (id)
    {
        case CONFIG_ID_FILLING_OPENING_TIME:
            return "FillingOpeningTime";
        case CONFIG_ID_VENTING_OPENING_TIME:
            return "VentingOpeningTime";
        case CONFIG_ID_MAIN_OPENING_TIME:
            return "MainOpeningTime";
        case CONFIG_ID_RELEASE_OPENING_TIME:
            return "ReleaseOpeningTime";
        case CONFIG_ID_DISCONNECT_OPENING_TIME:
            return "DisconOpeningTime";
        case CONFIG_ID_FILLING_MAX_APERTURE:
            return "FillingMaxAperture";
        case CONFIG_ID_VENTING_MAX_APERTURE:
            return "VentingMaxAperture";
        case CONFIG_ID_MAIN_MAX_APERTURE:
            return "MainMaxAperture";
        case CONFIG_ID_RELEASE_MAX_APERTURE:
            return "ReleaseMaxAperture";
        case CONFIG_ID_DISCONNECT_MAX_APERTURE:
            return "DisconMaxAperture";
        case CONFIG_ID_IGNITION_TIME:
            return "IgnitionTime";
        case CONFIG_ID_DEFAULT_OPENING_TIME:
            return "DefOpeningTime";
        case CONFIG_ID_DEFAULT_MAX_APERTURE:
            return "DefMaxAperture";
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

bool FileBackend::start() { return true; }

bool FileBackend::load(std::vector<uint8_t>& buf)
{
    std::ifstream is(path, std::ifstream::ate | std::ifstream::binary);
    if (!is.good())
        return false;

    size_t size = is.tellg();
    is.seekg(0);

    buf.resize(size);
    is.read(reinterpret_cast<char*>(buf.data()), size);

    return is.good();
}

bool FileBackend::save(std::vector<uint8_t>& buf)
{
    std::ofstream os(path, std::ifstream::binary);
    if (!os.good())
        return false;

    os.write(reinterpret_cast<char*>(buf.data()), buf.size());
    return os.good();
}
