#pragma once

#include <cstdint>
#include <reflect.hpp>
#include <string>

namespace Motor
{

enum class MEAControllerState : uint8_t
{
    INIT = 0,
    CALIBRATING,
    READY,
    ACTIVE,
    END
};

inline std::string to_string(MEAControllerState state)
{
    switch (state)
    {
        case MEAControllerState::INIT:
            return "INIT";
        case MEAControllerState::CALIBRATING:
            return "CALIBRATING";
        case MEAControllerState::READY:
            return "READY";
        case MEAControllerState::ACTIVE:
            return "ACTIVE";
        case MEAControllerState::END:
            return "END";
        default:
            return "UNKNOWN";
    }
}

struct MEAControllerStatus
{
    uint64_t timestamp;
    MEAControllerState state;

    MEAControllerStatus(uint64_t timestamp, MEAControllerState state)
        : timestamp(timestamp), state(state) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(MEAControllerStatus,
                          FIELD_DEF(timestamp) FIELD_DEF(state));
    }
};

}  // namespace Motor
