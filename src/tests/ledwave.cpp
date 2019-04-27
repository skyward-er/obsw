#include <miosix.h>
#include <stdio.h>

using namespace miosix;

typedef miosix::Gpio<GPIOG_BASE, 2> led1;
typedef miosix::Gpio<GPIOG_BASE, 3> led2;
typedef miosix::Gpio<GPIOD_BASE, 11> led3;

typedef miosix::Gpio<GPIOB_BASE, 1> led4;
typedef miosix::Gpio<GPIOC_BASE, 4> led5;
typedef miosix::Gpio<GPIOA_BASE, 4> led6;

int main()
{
     //   DeathStack board;
    led1::mode(Mode::OUTPUT);
    led2::mode(Mode::OUTPUT);
    led3::mode(Mode::OUTPUT);
    led4::mode(Mode::OUTPUT);
    led5::mode(Mode::OUTPUT);
    led6::mode(Mode::OUTPUT);
    int counter=0;
    int direction=0;

    printf("Started\n");

    while(1)
    {
        printf("Working\n");
        led1::high();
        led2::high();
        led3::high();
        led4::high();
        led5::high();
        led6::high();
        Thread::sleep(100);
        led1::low();
        led2::low();
        led3::low();
        led4::low();
        led5::low();
        led6::low();
        Thread::sleep(100);
    }
        /*
        if(counter==1)
        {
            led1::high();
            led2::low();
            led3::low();
            led4::low();
            led5::low();
            led6::low();
            direction=0;
        }
        else if(counter==2)
        {
            led1::low();
            led2::high();
            led3::low();
            led4::low();
            led5::low();
            led6::low();
        }
        else if(counter==3)
        {
            led1::low();
            led2::low();
            led3::high();
            led4::low();
            led5::low();
            led6::low();
        }
        else if(counter==4)
        {
            led1::low();
            led2::low();
            led3::low();
            led4::high();
            led5::low();
            led6::low();
        }
        else if(counter==5)
        {
            led1::low();
            led2::low();
            led3::low();
            led4::low();
            led5::high();
            led6::low();
        }
        else if(counter==6)
        {
            led1::low();
            led2::low();
            led3::low();
            led4::low();
            led5::low();
            led6::high();
            direction=1;
        }

        if(direction)
            counter--;
        else
            counter++;

        Thread::sleep(100);
    }
    */
}