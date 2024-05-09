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

#include <utils/Registry/RegistryFrontend.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace RIGv2
{

enum ConfigurationKeys
{
    CONFIG_ID_FILLING_OPENING_TIME    = 1,
    CONFIG_ID_VENTING_OPENING_TIME    = 2,
    CONFIG_ID_MAIN_OPENING_TIME       = 3,
    CONFIG_ID_RELEASE_OPENING_TIME    = 4,
    CONFIG_ID_DISCONNECT_OPENING_TIME = 5,
    CONFIG_ID_FILLING_MAX_APERTURE    = 6,
    CONFIG_ID_VENTING_MAX_APERTURE    = 7,
    CONFIG_ID_MAIN_MAX_APERTURE       = 8,
    CONFIG_ID_RELEASE_MAX_APERTURE    = 9,
    CONFIG_ID_DISCONNECT_MAX_APERTURE = 10,
    CONFIG_ID_IGNITION_TIME           = 11,
    CONFIG_ID_DEFAULT_OPENING_TIME    = 12,
    CONFIG_ID_DEFAULT_MAX_APERTURE    = 13,
};

const char* configurationIdToName(Boardcore::ConfigurationId id);

class Registry : public Boardcore::Module, public Boardcore::RegistryFrontend
{
public:
    Registry();

    [[nodiscard]] bool start();
};

class FileBackend : public Boardcore::RegistryBackend
{
public:
    explicit FileBackend(std::string path) : path{std::move(path)} {}

    [[nodiscard]] bool start() override;
    bool load(std::vector<uint8_t>& buf) override;
    bool save(std::vector<uint8_t>& buf) override;

private:
    std::string path;
};

}  // namespace RIGv2