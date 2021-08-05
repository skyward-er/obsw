#pragma once

static const int LEN_TEST = 169;

struct input_t
{
    float z;
    float vz;
    float vMod;
    float sampleTime;
};

struct output_t
{
    float z_setpoint;
    float vz_setpoint;
    float u;
    float deltas;
    int alpha;
};

struct test_t
{
    input_t input;
    output_t output;
};

extern const test_t DATA[LEN_TEST];
