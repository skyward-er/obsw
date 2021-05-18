/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio, Vincenzo Santomarco
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <miosix.h>

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "AeroBrakesController/AeroBrakesController.h"
#include "AeroBrakesController/AeroBrakesServo.h"
#include "events/Events.h"
#include "test_data.h"
#include "utils/testutils/TestHelper.h"

using namespace DeathStackBoard;

using namespace std;

void testAlgorithm();
void testAeroBrakes();
void wiggleServo();
void setServoFullyOpen();
void setServoFullyClosed();
void resetServo();
void manualServoControl();
void setServoParameters();
void resetServoParameters();
void waitUserInput();

class InputClass
{
public:
    float z;
    float vz;
    float vMod;
    uint64_t timestamp;
};

template <typename T>
class MockSensor : public Sensor<T>
{
private:
    int ts = 0;

protected:
    T sampleImpl() override
    {
        input_t input = DATA[ts].input;
        T sampled;

        sampled.z         = input.z;
        sampled.vz        = input.vz;
        sampled.vMod      = input.vMod;
        sampled.timestamp = ts;

        ts++;

        return sampled;
    }

    bool init() override { return true; }

    bool selfTest() override { return true; }
};

float minPosition   = AeroBrakesConfigs::AB_SERVO_MIN_POS;
float maxPosition   = AeroBrakesConfigs::AB_SERVO_MAX_POS;
float resetPosition = AeroBrakesConfigs::AB_SERVO_MIN_POS;

int main()
{
    sEventBroker->start();

    miosix::GpioPin pwmPin = miosix::GpioPin(GPIOC_BASE, 7);
    pwmPin.mode(miosix::Mode::ALTERNATE);
    pwmPin.alternateFunction(3);

    string temp;
    for (;;)
    {
        int choice;
        cout << "\n\nWhat do you want to do?:\n";
        cout << "1. Aerobrakes algorithm simulation (with test data)\n";
        cout << "2. Aerobrakes full extension\n";
        cout << "3. Servo wiggle\n";
        cout << "4. Servo fully open\n";
        cout << "5. Servo fully closed\n";
        cout << "6. Servo reset\n";
        cout << "7. Servo manual control\n";
        cout << "8. Set servo parameters\n";
        cout << "9. Reset servo parameters (default)\n";

        getline(cin, temp);
        stringstream(temp) >> choice;

        switch (choice)
        {
            case 1:
                testAlgorithm();
                break;
            case 2:
                testAeroBrakes();
                break;
            case 3:
                wiggleServo();
                break;
            case 4:
                setServoFullyOpen();
                break;
            case 5:
                setServoFullyClosed();
                break;
            case 6:
                resetServo();
                break;
            case 7:
                manualServoControl();
                break;
            case 8:
                setServoParameters();
                break;
            case 9:
                resetServoParameters();
                break;
            default:
                cout << "Invalid option \n";
                break;
        }
    }
}

void testAlgorithm()
{
    cout << "\n\n** AEROBRAKES ALGORITHM SIMULATION **\n\n";

    waitUserInput();

    // AeroBrakesController initialization
    MockSensor<InputClass> sensor;
    AeroBrakesServo servo{minPosition, maxPosition, resetPosition};
    AeroBrakesController<InputClass> aerobrakesController(sensor, &servo);

    // Start the state machine
    aerobrakesController.start();
    sensor.sample();
    EventCounter counter{*sEventBroker};
    counter.subscribe(TOPIC_FLIGHT_EVENTS);

    // Start test
    cout << "Starting aerobrakes test\n";
    sEventBroker->post({EV_LIFTOFF}, TOPIC_FLIGHT_EVENTS);
    while (counter.getCount(EV_SHADOW_MODE_TIMEOUT) < 1)
    {
        Thread::sleep(10);
    }

    for (int i = 1; i < LEN_TEST; i++)
    {
        sensor.sample();
        aerobrakesController.update();
        cout << "z: " << sensor.getLastSample().z
             << "\tvz: " << sensor.getLastSample().vz
             << "\tvMod: " << sensor.getLastSample().vMod
             << "\tservo: " << servo.getCurrentPosition() << "\n";
        Thread::sleep(AeroBrakesConfigs::UPDATE_TIME);
    }

    aerobrakesController.stop();
    counter.stop();

    cout << "\n\tTest finished!\n";
}

void testAeroBrakes()
{
    cout << "\n\n** AEROBRAKES FULL EXTENSION **\n\n";

    waitUserInput();

    // AeroBrakesController initialization
    MockSensor<InputClass> sensor{};
    AeroBrakesServo servo{minPosition, maxPosition, resetPosition};
    AeroBrakesController<InputClass> aerobrakesController{sensor, &servo};

    // Start the state machine
    aerobrakesController.start();
    EventCounter counter{*sEventBroker};
    counter.subscribe(TOPIC_ABK);

    // Start test
    cout << "Starting aerobrakes test\n";
    sEventBroker->post({EV_TEST_ABK}, TOPIC_ABK);

    while (counter.getCount(EV_TEST_TIMEOUT) < 1)
    {
        Thread::sleep(100);
    }

    aerobrakesController.stop();
    counter.stop();

    cout << "\n\tTest finished!\n";
}

void wiggleServo()
{
    string temp;
    cout << "\n\n** WIGGLE SERVO **\n\n";

    waitUserInput();

    AeroBrakesServo servo{minPosition, maxPosition, resetPosition};
    servo.enable();
    servo.reset();

    cout << "Wiggling ...\n";
    servo.selfTest();
    Thread::sleep(1000);    
    servo.disable();
    cout << "\n\tDone!\n";
}

void setServoFullyOpen()
{
    string temp;
    cout << "\n\n** SERVO FULLY OPEN **\n\n";

    waitUserInput();

    AeroBrakesServo servo{minPosition, maxPosition, resetPosition};
    servo.enable();
    servo.setMaxPosition();
    Thread::sleep(1000);
    servo.disable();

    cout << "\n\tDone!\n";
}

void setServoFullyClosed()
{
    string temp;
    cout << "\n\n** SERVO FULLY CLOSED **\n\n";

    waitUserInput();

    AeroBrakesServo servo{minPosition, maxPosition, resetPosition};
    servo.enable();
    servo.setMinPosition();
    Thread::sleep(1000);
    servo.disable();

    cout << "\n\tDone!\n";
}

void resetServo()
{
    string temp;
    cout << "\n\n** RESET SERVO **\n\n";

    waitUserInput();

    AeroBrakesServo servo{minPosition, maxPosition, resetPosition};
    servo.enable();
    servo.reset();
    Thread::sleep(1000);
    servo.disable();

    cout << "\n\tDone!\n";
}

void manualServoControl()
{
    cout << "\n\n** MANUAL SERVO CONTROL **\n\n";

    string temp;
    float angle;
    do
    {
        cout << "Write the servo postion (degrees):\n";
        getline(cin, temp);
        stringstream(temp) >> angle;
    } while (angle < minPosition || angle > maxPosition);

    AeroBrakesServo servo{minPosition, maxPosition, resetPosition};

    servo.enable();
    while (servo.getCurrentPosition() != angle)
    {
        servo.set(angle);
        Thread::sleep(AeroBrakesConfigs::UPDATE_TIME);
    }
    Thread::sleep(1000);
    servo.disable();

    cout << "\n\tDone!\n";
}

void setServoParameters()
{
    cout << "\n\n** SET SERVO PARAMETERS **\n\n";

    string temp;
    do
    {
        cout << "Write the servo minimum postion (degrees):\n";
        getline(cin, temp);
        stringstream(temp) >> minPosition;
    } while (minPosition < 0 || minPosition > 180.0f);

    do
    {
        cout << "Write the servo maximum postion (degrees):\n";
        getline(cin, temp);
        stringstream(temp) >> maxPosition;
    } while (maxPosition < 0 || maxPosition > 180.0f);

    do
    {
        cout << "Write the servo reset postion (degrees):\n";
        getline(cin, temp);
        stringstream(temp) >> resetPosition;
    } while (resetPosition < 0 || resetPosition > 180.0f);

    cout << "Configured parameteres:\n";
    cout << "\tminimum position: " << minPosition << "\n";
    cout << "\tmaximum position: " << maxPosition << "\n";
    cout << "\treset position: " << resetPosition << "\n";
}

void resetServoParameters()
{
    minPosition   = AeroBrakesConfigs::AB_SERVO_MIN_POS;
    maxPosition   = AeroBrakesConfigs::AB_SERVO_MAX_POS;
    resetPosition = AeroBrakesConfigs::AB_SERVO_MIN_POS;

    cout << "Configured parameteres (default):\n";
    cout << "\tminimum position: " << minPosition << "\n";
    cout << "\tmaximum position: " << maxPosition << "\n";
    cout << "\treset position: " << resetPosition << "\n";
}

void waitUserInput()
{
    string temp;
    do
    {
        cout << "Write 'start' to begin the test:\n";
        getline(cin, temp);
    } while (temp != "start");
}