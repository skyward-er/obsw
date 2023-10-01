/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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

#include <iostream>

namespace RIG
{
/**
 * @brief Median filter for embedded adapted to avoid recursion.
 * The object calculates a static 1D median filter, which depends on the
 * template parameters.
 *
 * @tparam T type of object to sort and median filter.
 * @tparam S number of samples needed for a static analysis.
 */
template <typename T, int S>
class MedianFilter
{
public:
    MedianFilter() : m_idx(0), ready(false), m_med(0)
    {
        // Initialize the arrays
        for (int i = 0; i < S; i++)
        {
            m_buf[i] = 0;
            m_tmp[i] = 0;
        }
    }

    /**
     * @brief Adds the sample S to the window and computes the
     * median if enough samples have been gathered.
     *
     * @param s The sample to add to the median filter
     */
    void add(T s)
    {
        m_buf[m_idx] = s;
        m_idx        = (m_idx + 1) % S;

        if (m_idx == 0)
        {
            calcMedian();
            ready = true;
        }
        else
        {
            ready = false;
        }
    }

    /**
     * @brief returns the median computed when the last sample was
     * added. Does not return anything meaningful if not enough samples
     * have been gathered; check isReady() first.
     *
     * @return T the median element
     */
    T getMedian() { return m_med; }

    /**
     * @brief returns the state of the computed median.
     * If the number of elements is not a multiple of the size,
     * then ready is false and the median does not have sense with
     * respect to the last inserted values.
     *
     * @note depending on the history of insertions, even if it is not ready,
     * the median is consistent. In case of not previously inserted values it is
     * 0 and in case of previously inserted values, the median is the previous
     * one.
     *
     * @return true
     * @return false
     */
    bool isReady() { return ready; }

    /**
     * @brief Resets the internal state of the object.
     */
    void reset()
    {
        m_idx = 0;
        m_med = 0;
        ready = false;

        // Reset the arrays
        for (int i = 0; i < S; i++)
        {
            m_buf[i] = 0;
            m_tmp[i] = 0;
        }
    }

private:
    int m_idx;
    bool ready;
    T m_buf[S], m_tmp[S], m_med;

    /**
     * @brief helper to calculate the median. Copies
     * the buffer into the temp area, then calls Hoare's in-place
     * selection algorithm to obtain the median.
     */
    void calcMedian()
    {
        for (int i = 0; i < S; i++)
        {
            m_tmp[i] = m_buf[i];
        }
        m_med = select(0, S - 1, S / 2);
    }

    /**
     * @brief partition function, like from quicksort.
     * l and r are the left and right bounds of the array (m_tmp),
     * respectively, and p is the pivot index.
     *
     * @param l left bound of m_tmp array
     * @param r right bound of m_tmp array
     * @param p pivot index
     */
    int partition(int l, int r, int p)
    {
        T tmp;
        T pv     = m_tmp[p];
        m_tmp[p] = m_tmp[r];
        m_tmp[r] = pv;
        int s    = l;
        for (int i = l; i < r; i++)
        {
            if (m_tmp[i] < pv)
            {
                tmp      = m_tmp[s];
                m_tmp[s] = m_tmp[i];
                m_tmp[i] = tmp;
                s++;
            }
        }
        tmp      = m_tmp[s];
        m_tmp[s] = m_tmp[r];
        m_tmp[r] = tmp;
        return s;
    }

    /**
     * @brief Hoare's quickselect. l and r are the
     * array bounds, and k conveys that we want to return
     * the k-th value
     *
     * @param l left bound of m_tmp array
     * @param r right bound of m_tmp array
     * @param k wanted position of return value
     */
    T select(int l, int r, int k)
    {
        int p = 0;
        do
        {
            if (l == r)
            {
                return m_tmp[l];
            }

            p = (l + r) / 2;
            p = partition(l, r, p);

            // The case in which k == p is evaluated outside the loop
            if (k < p)
            {
                r = p - 1;
            }
            else if (k > p)
            {
                l = p + 1;
            }

        } while (p != k);

        return m_tmp[k];
    }
};
}  // namespace RIG
