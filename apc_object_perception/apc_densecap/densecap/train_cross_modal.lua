--[[
Trains a depth CNN using cross modal distillation using the technique described in
Cross Modal Distillation for Supervision Transfer
Saurabh Gupta, Judy Hoffman, Jitendra Malik
https://arxiv.org/abs/1507.00448
--]]

require 'nn'
require 'cutorch'
require 'cunn'
require 'cudnn'
require 'densecap.DenseCapModel'
require 'densecap.DataLoader'
require 'densecap.optim_updates'

local cmd = torch.CmdLine()

-- Model options
cmd:option('-rgb', 'data/models/densecap/densecap-pretrained-vgg16.t7')
cmd:option('-output', 'data/models/densecap/depth.t7')
cmd:option('-data_h5', 'data/apc-depth-split-split00.txt.h5')
cmd:option('-data_json', 'data/apc-depth-split-split00.txt.json')
cmd:option('-learning_rate', 1e-5, 'learning rate to use')
cmd:option('-optim_beta1', 0.9, 'beta1 for adam')
cmd:option('-optim_beta2', 0.999, 'beta2 for adam')
cmd:option('-optim_epsilon', 1e-8, 'epsilon for smoothing')

local opt = cmd:parse(arg)

local dtype = 'torch.CudaTensor'

-- initialize the data loader class
opt.depth = 1
local loader = DataLoader(opt)
opt.label_strings = loader.info.label_strings
opt.num_classes = #opt.label_strings

print('number of classes: ', opt.num_classes)

-- Extract reference RGB model from densecap checkpoint
local rgb_dc_model = torch.load(opt.rgb).model
local conv1 = rgb_dc_model.nets.conv_net1
local conv2 = rgb_dc_model.nets.conv_net2

-- Reassemble the model
local rgb = nn.Sequential()
rgb:add(conv1)
rgb:add(conv2)

rgb:evaluate()
rgb:cuda()
cudnn.convert(rgb, cudnn)

-- Initialize depth network with same weights
local depth = rgb:clone()
depth:training()

local crit = nn.MSECriterion():cuda()

local depth_params, depth_grad_params = depth:getParameters()

local function lossFun()
  -- Fetch data using the loader
  local timer = torch.Timer()
  local info
  local data = {}
  local loaderOpt = {transform=true, iterate=true}
  data.image, data.gt_boxes, data.gt_labels, info, data.region_proposals = loader:getBatch(loaderOpt)
  transformed = info[1].transformed
  for k, v in pairs(data) do
    data[k] = v:cuda()
  end

  -- Split 6D image into RGB and HHA
  local rgb_input = data.image:narrow(2,1,3)
  local depth_input = data.image:narrow(2,4,3)

  -- Run the RGB net forward to get the target
  local rgb_output = rgb:forward(rgb_input)

  -- Run the HHA net forward to get the current output
  local depth_output = depth:forward(depth_input)

  -- Evaluate the L2 crit
  local loss = crit:forward(depth_output, rgb_output)
  local crit_grad = crit:backward(depth_output, rgb_output)

  -- and backpropagate the gradient.
  local grad = depth:backward(depth_input, crit_grad)

  return loss
end

local function evaluate()
  print("Evaluating...")

  depth:evaluate()
  local counter = 0
  local loss = 0
  while true do
    counter = counter + 1

    -- Grab a batch of data and convert it to the right dtype
    local loader_kwargs = {split=2, iterate=true}
    local img, gt_boxes, gt_labels, info, _ = loader:getBatch(loader_kwargs)
    local data = {
      image = img:type(dtype),
      gt_boxes = gt_boxes:type(dtype),
      gt_labels = gt_labels:type(dtype),
    }

    local rgb_input = data.image:narrow(2,1,3)
    local depth_input = data.image:narrow(2,4,3)

    local target = rgb:forward(rgb_input)
    local y = depth:forward(depth_input)
    loss = loss + crit:forward(y, target)

    if info[1].split_bounds[1] == info[1].split_bounds[2] then break end
  end
  loss = loss / counter
  print(iter, '?', loss)

  print("Saving checkpoint...")
  depth:clearState()
  depth:float()
  torch.save(opt.output, depth)
  depth:cuda()
  depth_params, depth_grad_params = depth:getParameters()
  print("Save done.")

  depth:training()
end

local optim_state = {}

local iter = 1

-- Training loop
while true do
  depth_grad_params:zero()

  local loss = lossFun()
  print(iter, loss)

  -- Parameter update
  adam(depth_params, depth_grad_params, opt.learning_rate, opt.optim_beta1, opt.optim_beta2, opt.optim_epsilon, optim_state)

  if (iter == 1) or (iter % 200 == 0) then
    evaluate()

    if iter == 6000 then
      break
    end
  end

  iter = iter + 1
end
