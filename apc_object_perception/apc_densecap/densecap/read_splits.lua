require 'torch'
require 'nn'

require 'densecap.DataLoader'
require 'densecap.DetectionModel'

--[[
  Read out train / val / test split from hdf5 / json and print them
  To be used in conjunction with segmentation
--]]

local utils = require 'densecap.utils'
local eval_utils = require 'eval.det_eval_utils'

local cmd = torch.CmdLine()
cmd:option('-checkpoint',
  'data/models/densecap/densecap-pretrained-vgg16.t7',
  'The checkpoint to evaluate')
cmd:option('-data_h5', 'data/centauro.h5', 'The HDF5 file to load data from; optional.')
cmd:option('-data_json', 'data/centauro-dicts.json', 'The JSON file to load data from; optional.')
cmd:option('-gpu', 0, 'The GPU to use; set to -1 for CPU')
cmd:option('-use_cudnn', 1, 'Whether to use cuDNN backend in GPU mode.')
cmd:option('-split', 'val', 'Which split to evaluate; either val or test.')
cmd:option('-max_images', -1, 'How many images to evaluate; -1 for whole split')
cmd:option('-rpn_nms_thresh', 0.7)
cmd:option('-final_nms_thresh', 0.3)
cmd:option('-num_proposals', 1000)
local opt = cmd:parse(arg)

-- First load the model
local checkpoint = torch.load(opt.checkpoint)
-- local model = checkpoint.model
print 'Loaded model'

local dtype, use_cudnn = utils.setup_gpus(opt.gpu, opt.use_cudnn)
print(string.format('Using dtype "%s"', dtype))

-- model:convert(dtype, use_cudnn)
-- model:setTestArgs{
--   rpn_nms_thresh=opt.rpn_nms_thresh,
--   final_nms_thresh=opt.final_nms_thresh,
--   max_proposals=opt.num_proposals,
-- }

-- Set up the DataLoader; use HDF5 and JSON files from checkpoint if they were
-- not explicitly provided.
if opt.data_h5 == '' then
  opt.data_h5 = checkpoint.opt.data_h5
end
if opt.data_json == '' then
  opt.data_json = checkpoint.opt.data_json
end

local loader = DataLoader(opt)

-- Actually run evaluation
local kwargs = {
  model=model,
  loader=loader,
  split=opt.split,
  max_images=opt.max_images,
  dtype=dtype,
}
local max_images = utils.getopt(kwargs, 'max_images', -1)
local split = utils.getopt(kwargs, 'split', 'val')
local split_to_int = {train=0, val=1, test=2}
split = split_to_int[split]
print('using split ', split)

-- model:evaluate()
loader:resetIterator(split)
  
local counter = 0
  while true do
    counter = counter + 1
    
    -- Grab a batch of data and convert it to the right dtype
    
    local data = {}
    local loader_kwargs = {split=split, iterate=true}
    local img, gt_boxes, gt_labels, info, _ = loader:getBatch(loader_kwargs)
    info = info[1] -- Since we are only using a single image
    
    -- Print a message to the console
    local msg = 'Processed image %s (%d / %d) of split %d'
    local num_images = info.split_bounds[2]
    if max_images > 0 then num_images = math.min(num_images, max_images) end
    print(string.format(msg, info.filename, counter, num_images, split))

    -- Break out if we have processed enough images
    if max_images > 0 and counter >= max_images then break end
    if info.split_bounds[1] == info.split_bounds[2] then break end
    
  end