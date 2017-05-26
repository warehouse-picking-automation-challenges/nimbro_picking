#!/bin/bash

VARIANT="-highres-depth"

for split in $*; do
	stdbuf -oL th train_cross_modal.lua -data_h5 data/apc${VARIANT}-split-${split}.h5 -data_json data/apc${VARIANT}-split-${split}.json -output data/models/densecap/depth${VARIANT}-${split}.t7 |& tee data/cross${VARIANT}-${split}.log
done

