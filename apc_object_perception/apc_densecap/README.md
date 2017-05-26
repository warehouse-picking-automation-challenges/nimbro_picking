
apc_densecap
============

This is an object detection framework developed for the APC 2016, based on the
very nice DenseCap framework released by Justin Johnson:
https://github.com/jcjohnson/densecap

See the `densecap/` subdirectory for the actual torch-based deep learning code
written in Lua.

The ROS code in `src/` contains a ROS wrapper for the Torch modules.
Notice that the Torch code was developed further after the APC, so that the ROS
wrapper does not support the finetuned models - rather, it uses the pretrained
DenseCap model for feature extraction and then uses liblinear to perform
SVM classification.
