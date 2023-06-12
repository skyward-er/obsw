#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

#include "Converter.h"

using namespace miosix;
using namespace Boardcore;

inline float randf() { return (std::rand() % 200 - 100) / 100.f; }

int main()
{
    constexpr int N = 10000;

    printf("Starting test\n");
    uint64_t start = TimestampTimer::getTimestamp();

    for (int i = 0; i < N; i++)
    {
        NEDCoords coords     = {randf(), randf(), randf()};
        AntennaAngles angles = rocketPositionToAntennaAngles(coords);
        printf("NED: %.2f ; %.2f ; %.2f -> Angles %.2f ; %.2f\n", coords.n,
               coords.e, coords.d, angles.theta1, angles.theta2);
    }

    uint64_t end = TimestampTimer::getTimestamp();
    printf("Took %llu millis for %d calls.\n", (end - start) / 1000ull, N);

    return 0;
}
