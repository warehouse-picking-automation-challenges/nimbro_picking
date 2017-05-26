require 'torch'
require 'nn'
require 'image'

require 'densecap.DenseCapModel'
local utils = require 'densecap.utils'
local box_utils = require 'densecap.box_utils'
local vis_utils = require 'densecap.vis_utils'


--[[
Run a trained DenseCap model on images.

The inputs can be any one of:
- a single image: use the flag '-input_image' to give path
- a directory with images: use flag '-input_dir' to give dir path
- MSCOCO split: use flag '-input_split' to identify the split (train|val|test)

The output can be controlled with:
- max_images: maximum number of images to process. Set to -1 to process all
- output_dir: use this flag to identify directory to write outputs to
- output_vis: set to 1 to output images/json to the vis directory for nice viewing in JS/HTML
--]]

-- local cmd = torch.CmdLine()
--
-- -- Model options
-- cmd:option('-checkpoint',
--   'data/models/densecap/densecap-pretrained-vgg16.t7')
-- cmd:option('-image_size', 720)
-- cmd:option('-rpn_nms_thresh', 0.7)
-- cmd:option('-final_nms_thresh', 0.3)
-- cmd:option('-num_proposals', 1000)
--
-- -- Input settings
-- cmd:option('-input_image', '',
--   'A path to a single specific image to caption')
-- cmd:option('-input_dir', '', 'A path to a directory with images to caption')
-- cmd:option('-input_split', '',
--   'A VisualGenome split identifier to process (train|val|test)')
--
-- -- Only used when input_split is given
-- cmd:option('-splits_json', 'info/densecap_splits.json')
-- cmd:option('-vg_img_root_dir', '', 'root directory for vg images')
--
-- -- Output settings
-- cmd:option('-max_images', 100, 'max number of images to process')
-- cmd:option('-output_dir', '')
--     -- these settings are only used if output_dir is not empty
--     cmd:option('-num_to_draw', 10, 'max number of predictions per image')
--     cmd:option('-text_size', 2, '2 looks best I think')
--     cmd:option('-box_width', 2, 'width of rendered box')
-- cmd:option('-output_vis', 1,
--   'if 1 then writes files needed for pretty vis into vis/ ')
-- cmd:option('-output_vis_dir', 'vis/data')
--
-- -- Misc
-- cmd:option('-gpu', 0)
-- cmd:option('-use_cudnn', 1)
-- local opt = cmd:parse(arg)

local densecapCudaDevice = 0
local dtype, use_cudnn = nil, nil

function load_model(checkpoint_file, gpu)
  densecapCudaDevice = gpu
  dtype, use_cudnn = utils.setup_gpus(densecapCudaDevice, 1) -- FIXME: enable cudnn!

  print("CUDA device during load: ", cutorch.getDevice())

  local checkpoint = torch.load(checkpoint_file)
  model = checkpoint.model
  model:convert(dtype, use_cudnn)
  model:setTestArgs{
    rpn_nms_thresh = 0.7,
    final_nms_thresh = 0.0,
    num_proposals = 1000,
  }
  model:evaluate()
end

function run_image(img, additionalProposals)
  cutorch.setDevice(densecapCudaDevice+1)
  print("CUDA device during prediction: ", cutorch.getDevice())

  -- Load, resize, and preprocess image
  local tmp = image.hflip(img)
  img = image.vflip(tmp)

  img = image.scale(img, 720):float()
  local H, W = img:size(2), img:size(3)
  local img_caffe = img:view(1, 3, H, W)
  img_caffe = img_caffe:index(2, torch.LongTensor{3, 2, 1}):mul(255)
  local vgg_mean = torch.FloatTensor{103.939, 116.779, 123.68}
  vgg_mean = vgg_mean:view(1, 3, 1, 1):expand(1, 3, H, W)
  img_caffe:add(-1, vgg_mean)

  -- Run the model forward
  local boxes, scores, captions = model:forward_test(img_caffe:type(dtype))

  local out_boxes = model.nets.localization_layer.output[2]:float():clone()
  local out_feats = model.nets.recog_net.forwardnodes[6].data.input[1]:float():clone()

  local out
  if(additionalProposals:dim() > 0 and additionalProposals:size(1) > 0) then
	model.nets.localization_layer.overrideBoxes = true
	model.nets.localization_layer.overrideBoxesData = additionalProposals:cuda()

	model.nets.localization_layer:setImageSize(H, W)
	model.nets.localization_layer:forward(model.nets.conv_net2.output)
	model.nets.recog_net:forward(model.nets.localization_layer.output)

	local add_out_boxes = model.nets.localization_layer.output[2]:float():clone()
	local add_out_feats = model.nets.recog_net.forwardnodes[6].data.input[1]:float():clone()

	model.nets.localization_layer.overrideBoxes = false

	out = {
		boxes = torch.cat(out_boxes, add_out_boxes, 1),
		features = torch.cat(out_feats, add_out_feats, 1)
	}
  else
    out = {
		boxes = out_boxes,
		features = out_feats
	}
  end

  return out
end

function run_image_original(img)
	cutorch.setDevice(densecapCudaDevice+1)

	-- Load, resize, and preprocess image
	local tmp = image.hflip(img)
	img = image.vflip(tmp)

	img = image.scale(img, 720):float()
	local H, W = img:size(2), img:size(3)
	local img_caffe = img:view(1, 3, H, W)
	img_caffe = img_caffe:index(2, torch.LongTensor{3, 2, 1}):mul(255)
	local vgg_mean = torch.FloatTensor{103.939, 116.779, 123.68}
	vgg_mean = vgg_mean:view(1, 3, 1, 1):expand(1, 3, H, W)
	img_caffe:add(-1, vgg_mean)

	-- Run the model forward
	local boxes, scores, captions = model:forward_test(img_caffe:type(dtype))

	local out_boxes = model.net.modules[3].output[2]:float():clone()
	local out_feats = model.net.modules[4].forwardnodes[6].data.input[1]:float():clone()

	local out = {
		boxes = out_boxes,
		features = out_feats
	}

	return out
end

function run_additional_boxes(boxes)
	cutorch.setDevice(densecapCudaDevice+1)

	model.nets.localization_layer.overrideBoxes = true
	model.nets.localization_layer.overrideBoxesData = additionalProposals:cuda()

	model.nets.localization_layer:forward(model.nets.conv_net2.output)
	model.nets.recognition_network:forward(model.nets.localization_layer.output)

	local add_boxes, add_scores, add_captions = model:forward_test(img_caffe:type(dtype))

	local add_out_boxes = model.net.modules[3].output[2]:float():clone()
	local add_out_feats = model.net.modules[4].forwardnodes[6].data.input[1]:float():clone()

	model.nets.localization_layer.overrideBoxes = false

	local out = {
		boxes = add_out_boxes,
		features = add_out_feats,
	}

	return out
end
