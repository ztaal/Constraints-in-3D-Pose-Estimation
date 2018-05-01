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

#ifndef YML_LOADER_H
#define YML_LOADER_H

// Covis
#include <covis/covis.h>

// YMAL loader_3d
#include "yaml-cpp/yaml.h"

namespace covis {
    namespace util {
        /**
         * @ingroup detect
         * @class yml_loader
         * @brief Class for loading ground truth poses
         *
         * This class is used to load ground truth poses from yml files
         * made specifically for the tejani dataset
         *
         * @author Martin Staal Steenberg
         */
        class yml_loader {
            public:

                /// Empty Constructor
                yml_loader() {};

                /// Empty destructor
                ~yml_loader() {};

                /**
                 * Load the ground truth poses from a yml file into a std::vector<std::vector<Eigen::Matrix4f> >
                 * @param vector vector Eigen matrix
                 */
                void load_gt( std::string filePath, std::vector<int> *objIds,
                                std::vector<std::vector<std::vector<Eigen::Matrix4f> > > *poses );

                /**
                 * Load the benchmark indices from a yml file into a std::vector<int> >
                 * @return vector int
                 */
                void load_benchmark( std::string filePath, std::string index, std::vector<int> *indices );

            private:

                /**
                 * Converts two std::vector<double> to an Eigen::Matrix4f
                 * @param rotation vector (size = 9)
                 * @param translation vector (size = 3)
                 * @return Eigen matrix
                 */
                inline Eigen::Matrix4f vec2eigen( std::vector<double> R, std::vector<double> T )
                {
                    Eigen::Matrix4f result;
                    result <<   R[0], R[1], R[2], T[0],
                                R[3], R[4], R[5], T[1],
                                R[6], R[7], R[8], T[2],
                                   0,    0,    0,    1;
                    return result;
                }
        };
    }
}

#endif
