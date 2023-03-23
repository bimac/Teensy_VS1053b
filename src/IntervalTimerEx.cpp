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

#include "IntervalTimerEx.h"

IntervalTimerEx::~IntervalTimerEx() { end(); }

void IntervalTimerEx::end() {
  callbacks[index] = nullptr;
  IntervalTimer::end();
}

// generate and preset the callback storage
callback_t IntervalTimerEx::callbacks[4]{
    nullptr,
    nullptr,
    nullptr,
    nullptr,
};

#if defined(USE_CPP11_CALLBACKS)

relay_t IntervalTimerEx::relays[4]{
    [] { callbacks[0](); },
    [] { callbacks[1](); },
    [] { callbacks[2](); },
    [] { callbacks[3](); },
};

#else

// generate the static array of relay functions
relay_t IntervalTimerEx::relays[4]{
    [] { callbacks[0](states[0]); },
    [] { callbacks[1](states[1]); },
    [] { callbacks[2](states[2]); },
    [] { callbacks[3](states[3]); },
};

// storage for the state values
void *IntervalTimerEx::states[4]{
    nullptr,
    nullptr,
    nullptr,
    nullptr,
};

#endif
