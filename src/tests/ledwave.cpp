#include <miosix.h>
#include <stdio.h>

typedef miosix::Gpio<GPIOG_BASE, 2>  led1;
typedef miosix::Gpio<GPIOG_BASE, 3>  led2;
typedef miosix::Gpio<GPIOD_BASE, 11> led3;

typedef miosix::Gpio<GPIOB_BASE, 1> led4;
typedef miosix::Gpio<GPIOC_BASE, 4> led5;
typedef miosix::Gpio<GPIOA_BASE, 4> led6;

int main()
{
    // DeathStack board;
    led1::mode(miosix::Mode::OUTPUT);
    led2::mode(miosix::Mode::OUTPUT);
    led3::mode(miosix::Mode::OUTPUT);
    led4::mode(miosix::Mode::OUTPUT);
    led5::mode(miosix::Mode::OUTPUT);
    led6::mode(miosix::Mode::OUTPUT);

    printf("Started\n");

    FILE* file = fopen("/sd/log.txt","w");

    if (file == NULL)
        printf("Error opening log file\n");

    while(1)
    {
        printf("Working\n");
        led1::high();
        led2::high();
        led3::high();
        led4::high();
        led5::high();
        led6::high();
        miosix::Thread::sleep(100);
        led1::low();
        led2::low();
        led3::low();
        led4::low();
        led5::low();
        led6::low();
        miosix::Thread::sleep(100);
    }
}