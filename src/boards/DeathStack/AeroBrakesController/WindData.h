#pragma once

#include <cstdint>
#include <string>
#include <ostream>

using std::ostream;
struct WindData
{
    uint64_t timestamp;
    float wind;

    static string header()
    {
        return "timestamp,wind\n";
    }

    void print(ostream& os)
    {
        os << timestamp << "," << wind << "\n";
    }
};