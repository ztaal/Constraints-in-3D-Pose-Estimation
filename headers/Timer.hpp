// Copyright (c) 2013, University of Southern Denmark
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the University of Southern Denmark nor the names of
//    its contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY OF SOUTHERN DENMARK BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef COVIS_CORE_TIMER_H
#define COVIS_CORE_TIMER_H

// Boost
#include <boost/shared_ptr.hpp>

// STL
#include <ostream>

// OpenCV
#include <opencv2/core/core.hpp>

namespace covis {
    namespace core {
        /**
         * @ingroup core
         * @class Timer
         * @brief Simple timer class for measuring the CPU time elapsed
         *
         *
         * Example:
         *
         * @code
         * void someFunction() {
         *   double t;
         *   Timer timer(__FUNCTION__); // Start timner
         *
         *   // Do some stuff...
         *
         *   t = timer.intermediate(); // Returns time since last call to intermediate()
         *
         *   // Do more stuff...
         *
         *   t = timer.seconds(); // Returns time since timer was started
         *
         *   // Now timer is destroyed
         * }
         * @endcode
         *
         * @author Martin Staal Steenberg
         */
        class Timer {
            public:
                /// Pointer type
                typedef boost::shared_ptr<Timer> Ptr;

                /**
                 * Constructor:
                 */
                Timer();

                /**
                 * Empty destructor:
                 */
                virtual ~Timer() {}

                /**
                 * Returns time elapsed, if possible, intermediate time between this call and last
                 * call to this function.
                 *
                 * @param desc a description of what was measured since last call to this function
                 * @return elapsed time in [s]
                 */
                double intermediate();

                /**
                 * Return time elapsed since start
                 *
                 * @param ticks end time in ticks, defaults to current tick cout
                 * @return elapsed time in [s]
                 */
                inline double seconds(int64 ticks = cv::getTickCount()) {
                    return double(ticks - _start) / cv::getTickFrequency();
                }

            protected:
                /// Start time, initialized in constructor
                int64 _start;

                /// Previous time, updated on each intermediate timer call
                int64 _previous;

                /**
                 * Return time elapsed since last call to @ref intermediate()
                 *
                 * @param ticks end time in ticks, defaults to current tick cout
                 * @return elapsed time in [s]
                 */
                inline double delta(int64 ticks = cv::getTickCount()) {
                    return double(ticks - _previous) / cv::getTickFrequency();
                }
        };
    }
}

#endif
