#!/bin/bash

CUDA_VISIBLE_DEVICES=2,3 qlua apc_train.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name 0805_tote_dc1 -data_file apc_lowres_0630_dc -train_split train -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 1 -suppress_x 0 -epochs 100 -dc_setting 1

CUDA_VISIBLE_DEVICES=2,3 qlua apc_train.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name 0805_tote -data_file apc_lowres_0630_dc -train_split train -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 1 -suppress_x 0 -epochs 100 -dc_setting 2

CUDA_VISIBLE_DEVICES=2,3 qlua apc_train.lua -verbose 2 -cuda_device 2 -cuda_device_train 1 -crop_mode 2 -num_classes 41 -obj_class 0 -setting tote -mode train -model_name 0805_tote_nodc -data_file apc_lowres_0630_dc -train_split train -feature_type hha -data_path /home/local/staff/milan/research/projects/APC/apc_data -densecap 0 -suppress_x 0 -epochs 100