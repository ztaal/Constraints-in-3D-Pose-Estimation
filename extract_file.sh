#!/bin/bash

from_path="$1"
to_path="$2"

files=(
15 19 40 59 66 84 98 103 104 106 112 128 129 135 141 144 145 157 176 202 205 209 214 228 245 268 272 274 278 285 310 316 323 326 343 345 350 358 361 368 372 374 399 403 405 416 430 431 459 469 476 477 480 495 499 502 508 549 553 585 605 608 625 650 651 655 658 662 678 688 691 696 701 734 741 767 784 796 810 860 862 875 885 902 914 938 940 958 966 968 971 974 976 985 992 1001 1002 1010 1019 1020 1047 1048 1056 1064 1066 1068 1069 1078 1087 1091 1094 1103 1120 1122 1123 1133 1146 1157 1167 1168 1169 1182 1191 1196 1199 1202 1205 1214 1216 1219 1224 1228 1230)

for i in ${files[@]}; do
    name=$(echo 00000$i | tail -c 5)
    extension=".ply"
    file=$name$extension
    path=$from_path$file
    cp $path $to_path
    printf "\r $i"
done

gt_file="../gt.yml"
gt_path=$from_path$gt_file
folder="../"
directory=$to_path$folder
cp $gt_path $directory

info_file="../info.yml"
info_path=$from_path$info_file
cp $info_path $directory
 
exit 0
