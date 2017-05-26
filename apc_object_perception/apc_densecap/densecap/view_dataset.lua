--[[
View images from the dataset
]]--

-------------------------------------------------------------------------------
-- Includes
-------------------------------------------------------------------------------
require 'torch'
require 'nngraph'
require 'optim'
require 'image'
require 'lfs'
require 'nn'
local cjson = require 'cjson'

require 'densecap.DataLoader'
require 'densecap.DetectionModel'
require 'densecap.optim_updates'
local utils = require 'densecap.utils'
local opts = require 'train_opts'
local models_detection = require 'models_detection'
local eval_utils = require 'eval.det_eval_utils'

-------------------------------------------------------------------------------
-- Initializations
-------------------------------------------------------------------------------
local opt = opts.parse(arg)
torch.setdefaulttensortype('torch.FloatTensor')
torch.manualSeed(opt.seed)

-- initialize the data loader class
local loader = DataLoader(opt)
opt.label_strings = loader.info.label_strings
opt.num_classes = #opt.label_strings

print('number of classes: ', opt.num_classes)

while true do
  -- Fetch data using the loader
  local timer = torch.Timer()
  local info
  local data = {}
  local loaderOpt = {transform=true, iterate=true}
  data.image, data.gt_boxes, data.gt_labels, info, data.region_proposals = loader:getBatch(loaderOpt)

  local perm = torch.LongTensor{3,2,1}
  local vis = data.image[1][{{1,3}}]:index(1, perm)
  rgb_win = image.display{image=vis, win=rgb_win}

  if opt.depth ~= 0 then
    local perm = torch.LongTensor{3,2,1}
    local vis = data.image[1][{{4,6}}]:index(1, perm)
    depth_win = image.display{image=vis, win=depth_win}
  end

  os.execute('sleep 3')
end
