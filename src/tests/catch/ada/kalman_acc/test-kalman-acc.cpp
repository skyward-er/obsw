
#ifdef STANDALONE_CATCH1_TEST
#include "catch/catch-tests-entry.cpp"
#endif

#define private public

#include <ADA/ADA.h>
#include <Common.h>
#include <configs/ADAconfig.h>

#include <iostream>
#include <random>
#include <sstream>
#include <utils/testutils/catch.hpp>

#include "test-kalman-acc-data.h"

using namespace DeathStackBoard;
using namespace ADAConfigs;

constexpr float NOISE_STD_DEV_P = 5;  // Noise varaince
constexpr float LSB_P           = 28;

constexpr float NOISE_STD_DEV_A = 7;  // Noise varaince
constexpr float LSB_A           = 0.0048;

ADA *ada;

unsigned seed = 1234567;                     // Seed for noise generation
std::default_random_engine generator(seed);  // Noise generator

float addNoise_a(float sample);
float quantization_a(float sample);
std::normal_distribution<float> distribution_a(0.0, NOISE_STD_DEV_A);

float addNoise_p(float sample);
float quantization_p(float sample);
std::normal_distribution<float> distribution_p(0.0, NOISE_STD_DEV_P);

typedef miosix::Gpio<GPIOA_BASE, 5> greenLed;

TEST_CASE("Testing Kalman with accelerometer")
{
    ReferenceValues ref_values;
    ref_values.ref_pressure = SIMULATED_PRESSURE[1];
    ref_values.ref_altitude = 0;
    ref_values.msl_pressure = SIMULATED_PRESSURE[1];

    ada = new ADA(ref_values, getKalmanConfig(), getKalmanAccConfig());

    ADAKalmanState state;
    unsigned int j = 0;
    for (unsigned int i = 0; i < DATA_SIZE_AX; i++)
    {
        // Send accelerometer data
        ada->updateAcc(addNoise_a(SIMULATED_AX[i]));

        // Send barometer data and check state at reduced rate
        if (i % 25 == 0 && j < DATA_SIZE_P - 1)
        {
            ada->updateBaro(addNoise_p(SIMULATED_PRESSURE[j]));
            j++;

            state = ada->getADAKalmanState();
            std::cout << state.x0_acc << ", " << state.x1_acc << ", "
                      << state.x2_acc << ", " << SIMULATED_PRESSURE[i] << ", "
                      << ada->last_acc_average << "\n";
            if (state.x0_acc == Approx(SIMULATED_Z[j]).margin(10))
            {
                SUCCEED();
            }
            else
            {
                FAIL("i = " << j << "\t\t" << state.x0_acc
                            << " != " << SIMULATED_Z[j]);
            }

            if (state.x1_acc == Approx(SIMULATED_VZ[j]).margin(10))
            {
                SUCCEED();
            }
            else
            {
                FAIL("i = " << j << "\t\t" << state.x1_acc
                            << " != " << SIMULATED_VZ[j]);
            }
        }
    }
}

float addNoise_p(float sample)
{
    float noise = distribution_p(generator);
    return quantization_p(sample + noise);
}

float addNoise_a(float sample)
{
    float noise = distribution_a(generator);
    return quantization_a(sample + noise);
}

float quantization_p(float sample) { return round(sample / LSB_P) * LSB_P; }
float quantization_a(float sample) { return round(sample / LSB_A) * LSB_A; }