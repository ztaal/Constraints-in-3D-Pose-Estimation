#!/bin/bash

../build/aout ../datasets/hinterstoisser/ -z --pose_prior --benchmark-sixd --ransac --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/01/ply_benchmark --save-dir=../data_hinterstoisser/ --save
../build/aout ../datasets/hinterstoisser/ -z --pose_prior --benchmark-sixd --ransac --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/02/ply_benchmark --save-dir=../data_hinterstoisser/ --save
../build/aout ../datasets/hinterstoisser/ -z --pose_prior --benchmark-sixd --ransac --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/03/ply_benchmark --save-dir=../data_hinterstoisser/ --save
../build/aout ../datasets/hinterstoisser/ -z --pose_prior --benchmark-sixd --ransac --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/04/ply_benchmark --save-dir=../data_hinterstoisser/ --save
../build/aout ../datasets/hinterstoisser/ -z --pose_prior --benchmark-sixd --ransac --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/05/ply_benchmark --save-dir=../data_hinterstoisser/ --save
../build/aout ../datasets/hinterstoisser/ -z --pose_prior --benchmark-sixd --ransac --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/06/ply_benchmark --save-dir=../data_hinterstoisser/ --save
../build/aout ../datasets/hinterstoisser/ -z --pose_prior --benchmark-sixd --ransac --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/07/ply_benchmark --save-dir=../data_hinterstoisser/ --save
../build/aout ../datasets/hinterstoisser/ -z --pose_prior --benchmark-sixd --ransac --object-dir=models --yml-file=gt.yml --benchmark-file=test_set_v1.yml --scene-dir=test/08/ply_benchmark --save-dir=../data_hinterstoisser/ --save 
