#!/bin/bash

set -e

TRAIN=0
TEST=1
DEPTH=1
DATASET="-crop-1470-raw"
VARIANT="-cross"

for split in $*; do
	echo "Processing split $split"
	if [[ $TRAIN -eq 1 ]]; then
		stdbuf -oL th train_detection.lua \
			-checkpoint_path data/centauro${DATASET}-checkpoint-${split}${VARIANT}.t7 \
			-data_h5 data/centauro${DATASET}-split-${split}.h5 \
			-data_json data/centauro${DATASET}-split-${split}.json \
			-depth $DEPTH \
			-depth_cnn data/models/densecap/centauro-depth${DATASET}-${split}.t7 \
			-cluster_anchors 1 \
			-max_iters 70000 \
			-finetune_rpn_after 1 \
			-evaluate_val 0 \
			-evaluate_train 0 \
			-weight_decay 10 \
			-eval_first_iteration 1 \
			-save_checkpoint_every 1000 |& tee data/centauro${DATASET}-split-${split}${VARIANT}.log
	fi

	if [[ $TEST -eq 1 ]]; then
		stdbuf -oL th evaluate_model.lua \
			-checkpoint data/centauro${DATASET}-checkpoint-${split}${VARIANT}.t7 \
			-informed 0 \
			-split test |& tee data/centauro${DATASET}-split-${split}${VARIANT}.result
	fi
done

