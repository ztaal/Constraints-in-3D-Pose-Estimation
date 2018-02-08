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


#ifndef COVIS_CORE_SCOPED_TIMER_H
#define COVIS_CORE_SCOPED_TIMER_H

// Own
#include "macros.h"

// Boost
#include <boost/shared_ptr.hpp>

// STL
#include <ostream>

// OpenCV
#include <opencv2/core/core.hpp>

namespace covis {
    namespace core {
        /**
         * @class ScopedTimer
         * @ingroup core
         * @brief Simple timer class for measuring the CPU time elapsed in a scope
         *
         * This timer prints a start message upon construction, optionally an intermediate time,
         * and finally an elapsed time upon destruction.
         *
         * Example:
         *
         * @code
         * void someFunction() {
         *   ScopedTimer timer(__FUNCTION__); // Prints a message containing the function name
         *
         *   // Do some stuff...
         *
         *   timer.intermediate("some stuff"); // Prints intermediate time and a message containing name and "some stuff"
         *
         *   // Do more stuff...
         *
         *   // Now timer is destroyed, and the total elapsed time and the function name is printed
         * }
         * @endcode
         *
         * If multiple instances of this class are created, they will print messages with increasing levels
         * of indentation.
         *
         * @author Anders Glent Buch
         */
        class ScopedTimer {
            public:
                /// Pointer type
                typedef boost::shared_ptr<ScopedTimer> Ptr;

                /**
                 * Constructor: set message to print before timing and output stream
                 * @param name timer name
                 * @param os output stream to print messages to
                 */
                ScopedTimer(const std::string& name = "", std::ostream& os = std::cout);

                /**
                 * Destructor: report elapsed time to stream
                 */
                virtual ~ScopedTimer();

                /**
                 * Report time elapsed so far to stream, and, if possible, intermediate time between this call and last
                 * call to this function.
                 *
                 * @param desc a description of what was measured since last call to this function
                 * @return elapsed time in [s]
                 */
                double intermediate(const std::string& desc = "");

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

                /// Stream to print messages to, defaults to std::cout
                std::ostream& _os;

                /// Timer name
                std::string _name;

                /// Global instance counter, initialized to zero
                static volatile int _instances;

                /// Local instance index
                int _idx;

                /**
                 * Convert to appropriate units
                 * @param s elapsed time in s
                 * @param elapsed output elapsed time in unit
                 * @param unit unit of elapsed time
                 */
                void convert(double s, double& elapsed, std::string& unit);

                /**
                 * Indent according to local instance index
                 */
                void indent(const std::string& indstring = "\t");

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
