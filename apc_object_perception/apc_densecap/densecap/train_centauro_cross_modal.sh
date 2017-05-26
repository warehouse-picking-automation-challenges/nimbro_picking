#!/bin/bash

VARIANT="-crop-720-raw"

for split in $*; do
	stdbuf -oL th train_cross_modal.lua -data_h5 data/centauro${VARIANT}-split-${split}.h5 -data_json data/centauro${VARIANT}-split-${split}.json -output data/models/densecap/centauro-depth${VARIANT}-${split}.t7 |& tee data/centauro-cross${VARIANT}-${split}.log
done

