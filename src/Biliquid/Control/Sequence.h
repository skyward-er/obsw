/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#pragma once

namespace Biliquid
{
class Actuators;
class SequenceManager;

enum class ControlSequence
{
    SEQUENCE_1 = 0,
    SEQUENCE_2,
    SEQUENCE_3,
};

struct Sequence
{
    Sequence(Actuators& actuators, SequenceManager& manager)
        : actuators(actuators), manager(manager)
    {
    }

    virtual void activate()   = 0;
    virtual void deactivate() = 0;

    bool pending = false;  //!< whether the sequence is pending execution
    bool state   = false;  //!< line state at the time of the last interrupt
    bool masked  = false;  //!< whether the sequence is masked (disabled)

    void setPending(bool state)
    {
        pending = true;
        this->state   = state;
    }

protected:
    Actuators& actuators;
    SequenceManager& manager;
};

struct Sequence1 : public Sequence
{
    using Sequence::Sequence;
    void activate() override;
    void deactivate() override;
};

struct Sequence2 : public Sequence
{
    using Sequence::Sequence;
    void activate() override;
    void deactivate() override;
};

struct Sequence3 : public Sequence
{
    using Sequence::Sequence;
    void activate() override;
    void deactivate() override;
};

}  // namespace Biliquid
