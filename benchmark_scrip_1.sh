#!/bin/bash

./build/aout ./datasets/hinterstoisser/ -z --pose_prior --benchmark-tejani --ransac --object-dir=models --yml-file=gt.yml --scene-dir=test/01/ply_benchmark --save 
./build/aout ./datasets/hinterstoisser/ -z --pose_prior --benchmark-tejani --ransac --object-dir=models --yml-file=gt.yml --scene-dir=test/02/ply_benchmark --save 
./build/aout ./datasets/hinterstoisser/ -z --pose_prior --benchmark-tejani --ransac --object-dir=models --yml-file=gt.yml --scene-dir=test/03/ply_benchmark --save 
./build/aout ./datasets/hinterstoisser/ -z --pose_prior --benchmark-tejani --ransac --object-dir=models --yml-file=gt.yml --scene-dir=test/04/ply_benchmark --save 
./build/aout ./datasets/hinterstoisser/ -z --pose_prior --benchmark-tejani --ransac --object-dir=models --yml-file=gt.yml --scene-dir=test/05/ply_benchmark --save 
./build/aout ./datasets/hinterstoisser/ -z --pose_prior --benchmark-tejani --ransac --object-dir=models --yml-file=gt.yml --scene-dir=test/06/ply_benchmark --save 
./build/aout ./datasets/hinterstoisser/ -z --pose_prior --benchmark-tejani --ransac --object-dir=models --yml-file=gt.yml --scene-dir=test/07/ply_benchmark --save 
./build/aout ./datasets/hinterstoisser/ -z --pose_prior --benchmark-tejani --ransac --object-dir=models --yml-file=gt.yml --scene-dir=test/08/ply_benchmark --save 

