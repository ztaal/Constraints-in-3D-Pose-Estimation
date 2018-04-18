#!/bin/bash

../build/aout ../datasets/tejani/ -z --pose_prior --benchmark-tejani --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/01/ply --save 
../build/aout ../datasets/tejani/ -z --pose_prior --benchmark-tejani --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/02/ply --save 
../build/aout ../datasets/tejani/ -z --pose_prior --benchmark-tejani --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/03/ply --save 
../build/aout ../datasets/tejani/ -z --pose_prior --benchmark-tejani --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/04/ply --save 
../build/aout ../datasets/tejani/ -z --pose_prior --benchmark-tejani --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/05/ply --save 
../build/aout ../datasets/tejani/ -z --pose_prior --benchmark-tejani --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/06/ply --save 


