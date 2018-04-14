#!/bin/bash

from_path="$1"
to_path="$2"

datasets=(01 02 03 04 05 06 07 08 09 10 11 12 13 14 15)

for i in ${datasets[@]}; do
    echo "Copying: "$i
    source=$from_path"test/"$i"/ply"
    target=$to_path"test/"$i"/ply"
    cp -R $source $target

    gt_source=$from_path"test/"$i"/gt.yml"
    file_target=$to_path"test/"$i"/"
    cp $gt_source $file_target
    
    info_source=$from_path"test/"$i"/info.yml"
    cp $info_source $file_target
done

