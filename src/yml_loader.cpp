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

void yml_loader::load_gt( std::string filePath, std::vector<int> *objIds,
                            std::vector<std::vector<std::vector<Eigen::Matrix4f> > > *poses )
{
    // Open file
    std::vector<YAML::Node> file = YAML::LoadAllFromFile( filePath );
    if (file[0].IsNull())
        COVIS_THROW("Cannot open file \"" << filePath << "\" for reading!");

    YAML::Node scene = file[0];

    // Find unique objects
    for(size_t i = 0; i < scene[0].size(); i++)
        (*objIds).push_back(scene[0][i]["obj_id"].as<int>());
    objIds->erase( std::unique( objIds->begin(), objIds->end() ), objIds->end() );

    // Resize vector
    poses->resize(scene.size());
    for (size_t i = 0; i < scene.size(); i++)
        (*poses)[i].resize(objIds->size());

    // Loop over scenes
    for (size_t i = 0; i < scene.size(); i++) {
        for (size_t j = 0; j < scene[i].size(); j++) {
            for (size_t k = 0; k < objIds->size(); k++) {
                if (scene[i][j]["obj_id"].as<int>() == (*objIds)[k]) {
                    std::vector<double> R = scene[i][j]["cam_R_m2c"].as<std::vector<double> >();
                    std::vector<double> T = scene[i][j]["cam_t_m2c"].as<std::vector<double> >();
                    (*poses)[i][k].push_back( vec2eigen(R, T) );
                }
            }
        }
	}
}

void yml_loader::load_benchmark( std::string filePath, std::string index, std::vector<int> *indices )
{
    // Open file
    std::vector<YAML::Node> file = YAML::LoadAllFromFile( filePath );
    if (file[0].IsNull())
        COVIS_THROW("Cannot open file \"" << filePath << "\" for reading!");

    // Loop over indices
    const YAML::Node& scene = file[0];
    index.erase(0, std::min(index.find_first_not_of('0'), index.size()-1));
    *indices = scene[index].as<std::vector<int> >();
}
