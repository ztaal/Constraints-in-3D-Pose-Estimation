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

#include "../headers/yml_loader.hpp"

using namespace covis::util;

void yml_loader::load( std::vector<std::vector<Eigen::Matrix4f> > *poses )
{
    // Open file
    std::vector<YAML::Node> file = YAML::LoadAllFromFile( filePath );
    if (file[0].IsNull())
        COVIS_THROW("Cannot open file \"" << filePath << "\" for reading!");

    // Loop over scenes
    YAML::Node scene = file[0];
    poses->resize(scene.size());
    for (size_t i = 0; i < scene.size(); i++) {
        for (size_t j = 0; j < scene[i].size(); j++) {
            std::vector<double> R = scene[i][j]["cam_R_m2c"].as<std::vector<double> >();
            std::vector<double> T = scene[i][j]["cam_t_m2c"].as<std::vector<double> >();
            (*poses)[i].push_back( vec2eigen(R, T) );
        }
	}
}