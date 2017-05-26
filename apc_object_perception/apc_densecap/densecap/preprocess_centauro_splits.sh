#!/bin/bash

dataset=$1
depth='%s_depth.png'
#depth=''
variant="-crop-720-raw"

for split in $dataset/splits/split*.txt; do
	split_name=$(basename $split)
	echo "Processing split $split_name"

	python preprocess_centauro.py --dataset $dataset --h5_output data/centauro$variant-split-$split_name.h5 --json_output data/centauro$variant-split-$split_name.json --test_set $split --depth "$depth" --image_size 720
done

