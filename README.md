# Constraints-in-3D-Pose-Estimation

This repository constains the code developed during a master project titled "Constraints in 3D Pose Estimation" at the University of Southern Denmark by Martin Staal Steenberg.

Quick installation
==================

The following guide has been tested on Ubuntu Linux 17.10.
All commands below should be entered in a terminal.

First download and install CoViS: https://gitlab.com/caro-sdu/covis.
Then download and install ymal-cpp: https://github.com/jbeder/yaml-cpp.

To download and compile this repository:

```sh
git clone https://github.com/ztaal/Constraints-in-3D-Pose-Estimation
mkdir -p Constraints-in-3D-Pose-Estimation/build
cd Constraints-in-3D-Pose-Estimation/build
cmake -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang -DCMAKE_BUILD_TYPE=Release ..
make
```

Run Code
====================
Example for benchmarking a sixd dataset:

```sh
./aout path/to/dataset/ --object-dir=/path/from/dataset/to/models --scene-dir=/path/from/dataset/to/ply --yml-file=gt.yml --benchmark-file=test_set_v1.yml --pose_prior --benchmark-sixd
```

