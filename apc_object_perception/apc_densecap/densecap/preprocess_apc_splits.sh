#!/bin/bash

dataset=$1
depth=feature_hhav.png
#depth=""
variant=highres-depth

#echo "WARNING: NO --require-capture!"

for split in data/split*.txt; do
	split_name=$(basename $split)
	echo "Processing split $split_name"

	# WARNING: Use --require-capture for shelf splits!
	python preprocess_apc.py --dataset $dataset --h5_output data/apc-$variant-split-$split_name.h5 --json_output data/apc-$variant-split-$split_name.json --test_set $split --val_split 0 --depth "$depth" --require-capture --image_size 1080
done

