#!/bin/bash

CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name 0805_tote_dc1 -data_file apc_lowres_0630_dc -train_split train -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 1 -suppress_x 0 -epochs 0 -dc_setting 1

CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name 0805_tote -data_file apc_lowres_0630_dc -train_split train -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 1 -suppress_x 0 -epochs 0 -dc_setting 2

CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name 0805_tote_nodc -data_file apc_lowres_0630_dc -train_split train -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -suppress_x 0 -epochs 0 -dc_setting 2

CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name 0805_tote_nodc -data_file apc_lowres_0630_dc -train_split train -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -suppress_x 0 -epochs 0 -dc_setting 3


# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name apc_hha_lowres_0629_train_011 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 1 -epochs 10
# 
# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name apc_hha_lowres_0629_train_011 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 1 -epochs 20
# 
# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name apc_hha_lowres_0629_train_011 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 1 -epochs 30
# 
# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name apc_hha_lowres_0629_train_011 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 1 -epochs 40
# 
# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name apc_hha_lowres_0629_train_011 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 1 -epochs 50
# 
# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name apc_hha_lowres_0629_train_011 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 1 -epochs 60
# 
# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name apc_hha_lowres_0629_train_011 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 1 -epochs 70
# 
# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name apc_hha_lowres_0629_train_011 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 1 -epochs 80
# 
# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name apc_hha_lowres_0629_train_011 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 1 -epochs 90
# 
# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name apc_hha_lowres_0629_train_011 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 1


# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode test -model_name apc_hha_lowres_0629_train_001 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 0 -epochs 10
# 
# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode test -model_name apc_hha_lowres_0629_train_001 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 0 -epochs 20
# 
# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode test -model_name apc_hha_lowres_0629_train_001 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 0 -epochs 30
# 
# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode test -model_name apc_hha_lowres_0629_train_001 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 0 -epochs 40
# 
# CUDA_VISIBLE_DEVICES=2,3 qlua apc_eval.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode test -model_name apc_hha_lowres_0629_train_001 -train_split train -data_file apc_lowres_0629 -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -learning_rate 0.001 -dropout 0.25 -suppress_x 0 -epochs 0


# qlua apc_eval.lua -verbose 2 -cuda_device 1 -num_classes 41 -obj_class 0 -setting shelf -mode test
# qlua apc_eval.lua -verbose 2 -cuda_device 1 -num_classes 41 -obj_class 0 -setting tote -mode test
# for c in $(seq 1 1 41)
# do
# 	echo $c
# 	# qlua apc_eval.lua -verbose 2 -cuda_device 1 -num_classes 3 -obj_class $c -setting shelf -mode test
# done;
# for c in $(seq 1 1 41)
# do
#         qlua apc_eval.lua -verbose 2 -cuda_device 3 -num_classes 3 -obj_class $c -setting tote -mode test
# done;
# 
# 
