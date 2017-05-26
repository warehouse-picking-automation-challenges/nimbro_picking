# APC Segmentation

This is the code for segmentation used for the Amazon Picking Challenge (APC) 2016. It is mostly based on the paper

[Husain et al., Combining Semantic and Geometric Features for Object Class Segmentation of Indoor Scenes, ICRA 2016]

The current version uses as input the RGB image and as additional features either depth only or the HHA features. The depth-only variant is used for shelf registration while the extended feature version is used for the final object segmentation.

The models for the shelf (44 classes) and tote (41 classes) are trained separately. The network requires the pretrained weights from the OverFeat architecture, which are saved in `models/mlp*`. These models require `cutorch` to be loaded.


## Installation

apc_segmentation is implemented in [Torch](http://torch.ch/), 
and depends on the following packages: 
[torch/torch7](https://github.com/torch/torch7), 
[torch/nn](https://github.com/torch/nn),
[torch/nngraph](https://github.com/torch/nngraph),
[torch/image](https://github.com/torch/image),
[soumith/matio-ffi.torch](https://github.com/soumith/matio-ffi.torch)
[LuaFileSystem](https://keplerproject.github.io/luafilesystem/manual.html)


After installing torch, you can install / update these dependencies by running the following:

```bash
sudo apt-get install libmatio2
luarocks install torch
luarocks install nn
luarocks install nngraph
luarocks install image
luarocks install luafilesystem
luarocks install matio
```

### (Recommended) GPU acceleration

If your have an NVIDIA GPU and want to accelerate the model with CUDA, you'll also need to install
[torch/cutorch](https://github.com/torch/cutorch) and [torch/cunn](https://github.com/torch/cunn);
you can install / update these by running:

```bash
luarocks install cutorch
luarocks install cunn
luarocks install cudnn
```

### (Optional) cuDNN

If you want to use NVIDIA's cuDNN library, you'll need to register for the CUDA Developer Program (it's free) and download the library from [NVIDIA's website](https://developer.nvidia.com/cudnn); you'll also need to install
the [cuDNN bindings for Torch](https://github.com/soumith/cudnn.torch) by running

```bash
luarocks install cudnn
```

## Running on new images

To run the model on one image, cd to `./lua` and use the `apc_test.lua` script. Assuming, a tote capture is in the folder `/tmp/tote/run_20160628_1704/image_008/`, and the trained model is in the file `/tmp/apc_data/apc_hha_lowres_0629_all_ds4_cl41_obj0_ep10_tote.t7`,  the full command should be something like

```bash
qlua apc_test.lua -cuda_device 1 -crop_mode 2 -num_classes 41 -setting tote -mode test -model_name apc_hha_lowres_0629_all -feature_type hha -data_path /tmp/apc_data -epochs 0 -im_dir /tmp/tote/run_20160628_1704/image_008/
```

### Pretrained model

You can download the model we used for APC 2016 by running the following script:

<!-- TODO: Insert correct URL in script -->
```bash
 sh scripts/download_apc2016_models.sh
```


## Training

To train a model, the `apc_train.lua` script should be used. Most parameters can be set directly from the command line.

### Evaluation

The script `apc_eval.lua` can be used to evaluate a particular model on a number of test images. The evaluation metrics can be printed out with `apc_summary.lua`.

### Dataset

A matlab script `matlab/createAPCDataset.m` is used to create a `.mat` file containing all data necessary to train a model.