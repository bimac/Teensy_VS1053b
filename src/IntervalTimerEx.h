// MIT License
//
// Copyright (c) 2020 luni64
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include "Arduino.h"

#define USE_CPP11_CALLBACKS // comment out if you want to use the traditional
                            // void pointer pattern to pass state to callbacks

#if defined(USE_CPP11_CALLBACKS)
#include <functional>
using callback_t = std::function<void()>;
using relay_t    = void (*)();

#else
using callback_t = void (*)(void *);
using relay_t    = void (*)();
#endif

class IntervalTimerEx : public IntervalTimer {
public:
  template <typename period_t> // begin is implemented as template to avoid
                               // replication the various versions of
                               // IntervalTimer::begin
#if defined(USE_CPP11_CALLBACKS)
  bool begin(callback_t callback, period_t period);
#else
  bool begin(callback_t callback, void *state, period_t period);
#endif

  void end();
  ~IntervalTimerEx();

protected:
  unsigned index = 0;
  static callback_t callbacks[4]; // storage for callbacks
  static relay_t relays[4];       // storage for relay functions
#if !defined(USE_CPP11_CALLBACKS)
  static void *states[4]; // storage for state variables
#endif
};

// Inline implementation ===============================================

#if defined(USE_CPP11_CALLBACKS)
template <typename period_t>
bool IntervalTimerEx::begin(std::function<void()> callback, period_t period) {
  for (index = 0; index < 4; index++) // find the next free slot
  {
    if (callbacks[index] == nullptr) // ->free slot
    {
      if (IntervalTimer::begin(
              relays[index],
              period)) // we got a slot but we need to also get an actual timer
      {
        callbacks[index] = callback; // if ok -> store callback
        return true;
      }
      return false;
    }
  }
  return false; // can never happen if bookkeeping is ok
}

#else // traditional void pointer pattern to pass state to callbacks

template <typename period_t>
bool IntervalTimerEx::begin(callback_t callback, void *state, period_t period) {
  for (index = 0; index < 4; index++) // find the next free slot
  {
    if (callbacks[index] == nullptr) // ->free slot
    {
      if (IntervalTimer::begin(
              relays[index],
              period)) // we got a slot but we need to also get an actual timer
      {
        callbacks[index] = callback; // if ok -> store callback...
        states[index]    = state;    // ...and state
        return true;
      }
      return false;
    }
  }
  return false; // can never happen if bookkeeping is ok
}

#endif
