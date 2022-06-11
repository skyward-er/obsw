/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <drivers/timer/PWM.h>
#include <miosix.h>

#define a3f 208  // 208 Hz
#define b3f 233  // 233 Hz
#define b3 247   // 247 Hz
#define c4 261   // 261 Hz MIDDLE C
#define c4s 277  // 277 Hz
#define e4f 311  // 311 Hz
#define f4 349   // 349 Hz
#define a4f 415  // 415 Hz
#define b4f 466  // 466 Hz
#define b4 493   // 493 Hz
#define c5 523   // 523 Hz
#define c5s 554  // 554 Hz
#define e5f 622  // 622 Hz
#define f5 698   // 698 Hz
#define f5s 740  // 740 Hz
#define a5f 831  // 831 Hz

#define rest -1

// Parts 1 and 2 (Intro)

int song1_intro_melody[] = {c5s, e5f, e5f, f5,   a5f, f5s, f5,
                            e5f, c5s, e5f, rest, a4f, a4f};
int song1_intro_rhythm[] = {6, 10, 6, 6, 1, 1, 1, 1, 6, 10, 4, 2, 10};

// Parts 3 or 5 (Verse 1)

int song1_verse1_melody[] = {
    rest, c4s,  c4s,  c4s, c4s, e4f, rest, c4,   b3f, a3f, rest, b3f, b3f,
    c4,   c4s,  a3f,  a4f, a4f, e4f, rest, b3f,  b3f, c4,  c4s,  b3f, c4s,
    e4f,  rest, c4,   b3f, b3f, a3f, rest, b3f,  b3f, c4,  c4s,  a3f, a3f,
    e4f,  e4f,  e4f,  f4,  e4f, c4s, e4f,  f4,   c4s, e4f, e4f,  e4f, f4,
    e4f,  a3f,  rest, b3f, c4,  c4s, a3f,  rest, e4f, f4,  e4f};

int song1_verse1_rhythm[] = {2, 1, 1, 1, 1, 2, 1, 1, 1, 5, 1, 1, 1, 1, 3, 1,
                             2, 1, 5, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 3,
                             1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 4, 5, 1, 1, 1,
                             1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 3, 1, 1, 1, 3};

const char* lyrics_verse1[] = {"We're ",
                               "no ",
                               "strangers ",
                               "",
                               "to ",
                               "love ",
                               "",
                               "\r\n",
                               "You ",
                               "know ",
                               "the ",
                               "rules ",
                               "and ",
                               "so ",
                               "do ",
                               "I\r\n",
                               "A ",
                               "full ",
                               "commitment's ",
                               "",
                               "",
                               "what ",
                               "I'm ",
                               "thinking ",
                               "",
                               "of",
                               "\r\n",
                               "You ",
                               "wouldn't ",
                               "",
                               "get ",
                               "this ",
                               "from ",
                               "any ",
                               "",
                               "other ",
                               "",
                               "guy\r\n",
                               "I ",
                               "just ",
                               "wanna ",
                               "",
                               "tell ",
                               "you ",
                               "how ",
                               "I'm ",
                               "feeling",
                               "\r\n",
                               "Gotta ",
                               "",
                               "make ",
                               "you ",
                               "understand",
                               "",
                               "\r\n"};

// Parts 4 or 6 (Chorus)

int song1_chorus_melody[] = {
    b4f, b4f, a4f, a4f, f5,  f5,  e5f, b4f, b4f, a4f,  a4f, e5f, e5f, c5s, c5,
    b4f, c5s, c5s, c5s, c5s, c5s, e5f, c5,  b4f, a4f,  a4f, a4f, e5f, c5s, b4f,
    b4f, a4f, a4f, f5,  f5,  e5f, b4f, b4f, a4f, a4f,  a5f, c5,  c5s, c5,  b4f,
    c5s, c5s, c5s, c5s, c5s, e5f, c5,  b4f, a4f, rest, a4f, e5f, c5s, rest};

int song1_chorus_rhythmn[] = {1, 1, 1, 1, 3, 3, 6, 1, 1, 1, 1, 3, 3, 3, 1,
                              2, 1, 1, 1, 1, 3, 3, 3, 1, 2, 2, 2, 4, 8, 1,
                              1, 1, 1, 3, 3, 6, 1, 1, 1, 1, 3, 3, 3, 1, 2,
                              1, 1, 1, 1, 3, 3, 3, 1, 2, 2, 2, 4, 8, 4};

const char* lyrics_chorus[] = {
    "Never ",  "",       "gonna ", "",       "give ",  "you ",    "up\r\n",
    "Never ",  "",       "gonna ", "",       "let ",   "you ",    "down",
    "",        "\r\n",   "Never ", "",       "gonna ", "",        "run ",
    "around ", "",       "",       "",       "and ",   "desert ", "",
    "you\r\n", "Never ", "",       "gonna ", "",       "make ",   "you ",
    "cry\r\n", "Never ", "",       "gonna ", "",       "say ",    "goodbye ",
    "",        "",       "\r\n",   "Never ", "",       "gonna ",  "",
    "tell ",   "a ",     "lie ",   "",       "",       "and ",    "hurt ",
    "you\r\n"};

using namespace miosix;
using namespace Boardcore;

constexpr float DUTY_CYCLE  = 0.5;
constexpr int MIN_FREQUENCY = 20;
constexpr int MAX_FREQUENCY = 20000;
constexpr int STEPS         = 100;

PWM buzz(TIM8, 4000);

void tone(int frequency, int delay)
{
    buzz.enableChannel(TimerUtils::Channel::CHANNEL_1);
    buzz.setDutyCycle(TimerUtils::Channel::CHANNEL_1, DUTY_CYCLE);
    buzz.setFrequency(frequency);
    delayMs(delay);
    buzz.disableChannel(TimerUtils::Channel::CHANNEL_1);
}

int main()
{
    volatile int beatLength      = 60;  // determines tempo
    float beatSeparationConstant = 0.3;

    unsigned int a          = 4;  // part index
    unsigned int b          = 0;  // song index
    unsigned int c          = 0;  // lyric index
    unsigned int noteLength = 0;

    for (int i = 0; i < 100; i++)
        printf("\n");

    buzz.enableChannel(TimerUtils::Channel::CHANNEL_1);
    buzz.setDutyCycle(TimerUtils::Channel::CHANNEL_1, DUTY_CYCLE);
    delayMs(5 * 1000);

    while (a != 7)
    {
        if (a == 1 || a == 2)
        {
            // intro
            noteLength = beatLength * song1_intro_rhythm[b];
            if (song1_intro_melody[b] > 0)
            {
                tone(song1_intro_melody[b], noteLength);
            }
            b++;
            if (b >= sizeof(song1_intro_melody) / sizeof(int))
            {
                a++;
                b = 0;
                c = 0;
            }
        }
        else if (a == 3 || a == 5)
        {
            // verse
            noteLength = beatLength * song1_verse1_rhythm[b];
            if (song1_verse1_melody[b] > 0)
            {
                printf(lyrics_verse1[c]);
                tone(song1_verse1_melody[b], noteLength);
                c++;
            }
            b++;
            if (b >= sizeof(song1_verse1_melody) / sizeof(int))
            {
                a++;
                b = 0;
                c = 0;
            }
        }
        else if (a == 4 || a == 6)
        {
            // chorus
            noteLength = beatLength * song1_chorus_rhythmn[b];
            if (song1_chorus_melody[b] > 0)
            {
                printf(lyrics_chorus[c]);
                tone(song1_chorus_melody[b], noteLength);
                c++;
            }
            b++;
            if (b >= sizeof(song1_chorus_melody) / sizeof(int))
            {
                printf("\n");
                a++;
                b = 0;
                c = 0;
            }
        }
        delayMs(noteLength);
        delayMs(noteLength * beatSeparationConstant);
    }

    while (true)
        Thread::sleep(1000);
}