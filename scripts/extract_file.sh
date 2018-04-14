#!/bin/bash

from_path="$1"
to_path="$2"

files=(2 7 11 15 23 27 32 33 39 43 45 48 58 61 65 69 72 78 80 83 87 92 96 103 105 110 119 122 128 131 134 136 139 141 145 152 156 166 170 175 177 185 194 202 206 211 214 217 231 244 248 250 259 265 273 276 282 285 289 295 298 300 315 318 321 323 329 334 354 358 361 364 367 370 374 383 388 396 402 404 409 413 426 429 434 439 442 455 462 465 469 472 475 479 485 492 496 499 501 503)

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
