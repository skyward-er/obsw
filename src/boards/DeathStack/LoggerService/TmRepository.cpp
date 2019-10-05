#include "TmRepository.h"

namespace DeathStackBoard
{

/* Global struct containing all telemetry packets. */
TmRepository_t tm_repository;

void initTelemetries()
{
    memset(&tm_repository, 0, sizeof(tm_repository));
}

}