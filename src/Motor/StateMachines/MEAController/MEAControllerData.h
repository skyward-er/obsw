#pragma once

#include <cstdint>
#include <reflect.hpp>

namespace Motor
{

enum class MEAControllerState
{
    INIT = 0,
    READY,
    ACTIVE,
    END
};

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
