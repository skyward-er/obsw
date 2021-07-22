#pragma once

#include <cstdint>
#include <ostream>
#include <string>

using std::ostream;
struct WindData
{
    uint64_t timestamp;
    float wind;

    static std::string header() { return "timestamp,wind\n"; }

    void print(ostream& os) { os << timestamp << "," << wind << "\n"; }
};