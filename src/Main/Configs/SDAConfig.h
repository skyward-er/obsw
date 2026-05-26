#pragma once

#include <units/Frequency.h>

namespace Main
{

namespace Config
{

namespace SDA
{
using namespace Boardcore::Units::Frequency;

constexpr Hertz UPDATE_RATE_SDA         = 100_hz;
constexpr float UPDATE_RATE_SDA_SECONDS = 1.0 / UPDATE_RATE_SDA.value();


};  // namespace SDA

};  // namespace Config

};  // namespace Main
