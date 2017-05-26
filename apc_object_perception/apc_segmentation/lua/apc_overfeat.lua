require 'torch'
--require 'libopencv24'
--require 'mattorch'
require 'nn'
--require 'nnx'
require 'optim'
require 'image'
require 'cunn'
require 'nngraph'

require 'util.misc'

local ParamBank = require 'ParamBank'

local SpatialConvolution = nn.SpatialConvolutionMM
local SpatialConvolutionMM = nn.SpatialConvolutionMM
local SpatialMaxPooling = nn.SpatialMaxPooling
local ReLU = nn.ReLU
local SpatialZeroPadding = nn.SpatialZeroPadding

nngraph.setDebug(true)  -- uncomment for debug mode
torch.setdefaulttensortype('torch.FloatTensor')
-- globals fs --> filter size
fs1 = 5
fs2 = 11
pad1 = (fs1-1)/2
pad2 = (fs2-1)/2
epochs = 200
offset = 0
function create_network0(downsample)

  if (downsample == 4) then
    d1=2
    d2=2
  elseif (downsample == 8) then
    d1 = 4
    d2 = 2
  elseif (downsample == 16) then
    d1 = 4
    d2 = 4
  elseif (downsample == 32) then
    d1 = 8
    d2 = 4
  end


  local net = nn.Sequential();  -- make a multi-layer structure
  net:add(SpatialConvolutionMM(3, 96, 11, 11, 1,1, (11-1)/2, (11-1)/2))
  net:add(nn.Threshold(0.000001, 0.00000))
  net:add(SpatialMaxPooling(d1,d1)) -- 32x240x320
  net:add(SpatialConvolutionMM(96, 256, 5, 5, 1, 1, (5-1)/2, (5-1)/2))
  net:add(nn.Threshold(0.000001, 0.00000))
  net:add(SpatialMaxPooling(d2,d2)) -- 32x120x160
  net = net:float()
  local m = net.modules
  print('==> overwrite network parameters with pre-trained weights')
  ParamBank:init(overfeatDir.."net_weight_0")
  ParamBank:read(        0, {96,3,11,11},    m[offset+1].weight)
  ParamBank:read(    34848, {96},            m[offset+1].bias)
  ParamBank:read(    34944, {256,96,5,5},    m[offset+4].weight)
  ParamBank:read(   649344, {256},           m[offset+4].bias)
  if params.cuda_device > 0 then net:cuda() end
  return net
end

function create_network_depth(downsample)
  local net = nn.Sequential();  -- make a multi-layer structure
  net:add(SpatialConvolutionMM(3, 96, 11, 11, 1,1, (11-1)/2, (11-1)/2))
  net:add(nn.Threshold(0.000001, 0.00000))
  net:add(SpatialMaxPooling(downsample,downsample)) -- 96x120x160
  net = net:float()
  local m = net.modules
  print('==> overwrite network parameters with pre-trained weights')
  ParamBank:init(overfeatDir.."net_weight_0")
  ParamBank:read(        0, {96,3,11,11},    m[offset+1].weight)
  ParamBank:read(    34848, {96},            m[offset+1].bias)
  if params.cuda_device > 0 then net:cuda() end
  return net
end
