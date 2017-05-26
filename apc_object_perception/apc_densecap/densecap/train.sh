#!/bin/bash
# kate: replace-tabs off; indent-width 4;

set -e

#. /home/local/stud/schwarzm/apc/torch/install/bin/torch-activate
#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64

TRAIN=1
TEST=1
DEPTH=1
DATASET="-highres-depth"
VARIANT="-cross"
OUT_DATASET="-highres-depth"
OUT_VARIANT="-cross"
PROPOSALS=0

# -max_iters 70000 -depth 0 -finetune_rpn_after 1 -save_checkpoint_every 1000 -evaluate_val 0 -evaluate_train 1 -weight_decay 10 -eval_first_iteration 1

for split in $*; do
	echo "Processing split $split"
	if [[ $TRAIN -eq 1 ]]; then
		th train_detection.lua \
			-checkpoint_path data/checkpoint${DATASET}-${split}${VARIANT}.t7 \
			-data_h5 data/apc${DATASET}-split-${split}.h5 \
			-data_json data/apc${DATASET}-split-${split}.json \
			-max_iters 70000 \
			-cluster_anchors 1 \
			-depth $DEPTH \
			-depth_cnn data/models/densecap/depth${DATASET}-${split}.t7 \
			-finetune_rpn_after 1 \
			-evaluate_val 0 \
			-evaluate_train 0 \
			-weight_decay 10 \
			-eval_first_iteration 1 \
			-backend cudnn \
			-save_checkpoint_every 1000 |& tee data/apc${DATASET}-split-${split}${VARIANT}.log
	fi

	if [[ $TEST -eq 1 ]]; then
		th evaluate_model.lua \
			-checkpoint data/checkpoint${DATASET}-${split}${VARIANT}.t7 \
			-add_proposals ${PROPOSALS} \
			-informed 0 \
			-split test |& tee data/apc${OUT_DATASET}-split-${split}${OUT_VARIANT}-uninformed.result

		th evaluate_model.lua \
			-checkpoint data/checkpoint${DATASET}-${split}${VARIANT}.t7 \
			-add_proposals ${PROPOSALS} \
			-informed 1 \
			-split test |& tee data/apc${OUT_DATASET}-split-${split}${OUT_VARIANT}-informed.result
	fi
done
