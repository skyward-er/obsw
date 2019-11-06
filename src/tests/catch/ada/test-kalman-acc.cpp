
#ifdef STANDALONE_CATCH1_TEST
#include "catch/catch-tests-entry.cpp"
#endif

#include <Common.h>
#include <DeathStack/ADA/ADA.h>
#include <iostream>
#include <random>
#include <sstream>
#include <utils/testutils/catch.hpp>
#include "test-kalman-acc-data.h"

using namespace DeathStackBoard;

constexpr float NOISE_STD_DEV_P = 5;  // Noise varaince
constexpr float LSB_P           = 28;

constexpr float NOISE_STD_DEV_A = 7;  // Noise varaince
constexpr float LSB_A           = 0.0048;

ADA *ada;
miosix::FastMutex ada_mutex;
void accelerometerThread(void* args);

unsigned seed = 1234567;  // Seed for noise generation
std::default_random_engine generator(seed);  // Noise generator

float addNoise_a(float sample);
float quantization_a(float sample);
std::normal_distribution<float> distribution_a(0.0, NOISE_STD_DEV_A);

float addNoise_p(float sample);
float quantization_p(float sample);
std::normal_distribution<float> distribution_p(0.0, NOISE_STD_DEV_P);

typedef miosix::Gpio<GPIOG_BASE, 13> greenLed;

TEST_CASE("Testing Kalman with accelerometer")
{
    ReferenceValues ref_values;
    ref_values.ref_pressure = SIMULATED_PRESSURE[1];
    ref_values.ref_altitude = 0;
    {
        miosix::Lock<miosix::FastMutex> l(ada_mutex);
        ada = new ADA(ref_values);
    }

    miosix::Thread* t_acc = miosix::Thread::create(accelerometerThread,1024);
    KalmanState state;
    for (unsigned int i = 0; i < DATA_SIZE_P; i++)
    {
        {
            miosix::Lock<miosix::FastMutex> l(ada_mutex);
            ada->updateAcc(SIMULATED_AX[0]);
            ada->updateBaro(SIMULATED_PRESSURE[i]);
            state = ada->getKalmanState();
        }
        // std::cout << state.x0_acc << "\n";
        if(i>100)
        {
            if (state.x0_acc == Approx(SIMULATED_Z[i]).margin(50))
            {
                SUCCEED();
            }
            else
            {
                FAIL("i = " << i << "\t\t" << state.x0_acc
                            << " != " << SIMULATED_Z[i]);
            }
        }
        if (i>100)
        {
            if (state.x1_acc == Approx(SIMULATED_VZ[i]).margin(20))
            {
                SUCCEED();
            }
            else
            {
                FAIL("i = " << i << "\t\t" << state.x1_acc
                            << " != " << SIMULATED_VZ[i]);
            }
        }
        
        miosix::Thread::sleep(50); // 20 Hz
    }
}

void accelerometerThread(void* args)
{
    for (unsigned int i = 0; i < DATA_SIZE_AX; i++)
    {
        miosix::Lock<miosix::FastMutex> l(ada_mutex);
        ada->updateAcc((SIMULATED_AX[i]-1)*9.81);
        miosix::Thread::sleep(2); // 500 Hz
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