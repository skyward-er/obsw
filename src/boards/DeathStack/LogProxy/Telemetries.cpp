#include "Telemetries.h"

namespace DeathStackBoard
{

Telemetries tm_repository;

void initTelemetries()
{
    memset(&tm_repository, 0, sizeof(tm_repository));
}

}