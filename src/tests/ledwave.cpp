#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <stdio.h>

using led1 = miosix::leds::led_red1;
using led2 = miosix::leds::led_red2;
using led3 = miosix::leds::led_blue1;

/* NOTE:
 * These are conencted to the enable pin of the thermal
 * cutters and the cs of the lis3mdl magnetometer
 */
using led4 = miosix::leds::led_blue2;
using led5 = miosix::leds::led_green1;
using led6 = miosix::leds::led_green2;

// Test timeout
constexpr int TEST_TIMEOUT = 10;  // seconds

int main()
{
    printf("Waving!\n");

    // Sampling
    for (int i = 0; i < TEST_TIMEOUT; i++)
    {
        // On
        led3::high();
        miosix::delayMs(500 / 6);
        led1::high();
        miosix::delayMs(500 / 6);
        led5::high();
        miosix::delayMs(500 / 6);
        led6::high();
        miosix::delayMs(500 / 6);
        led2::high();
        miosix::delayMs(500 / 6);
        led4::high();
        miosix::delayMs(500 / 6);

        // Off
        led3::low();
        miosix::delayMs(500 / 6);
        led1::low();
        miosix::delayMs(500 / 6);
        led5::low();
        miosix::delayMs(500 / 6);
        led6::low();
        miosix::delayMs(500 / 6);
        led2::low();
        miosix::delayMs(500 / 6);
        led4::low();
        miosix::delayMs(500 / 6);
    }

    return 0;
}