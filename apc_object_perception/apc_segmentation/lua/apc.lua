require 'torch'		-- duh
require 'nn'		-- neural networks
require 'nngraph'	-- modular network support
require 'optim'		-- optimization
require 'image'		-- image processing, load, save, scale, ...
-- require 'cunn'	-- cuda. required for GPU train/test (via setup_gpus(..))
require 'lfs'		-- file system: create dir, etc.
require 'util.misc'	-- Anton's helper functions

--[[
Test image segmentation based on
[Husain et al., Combining Semantic and Geometric Features 
for Object Class Segmentation of Indoor Scenes, ICRA 2016]

--]]


-- nngraph.setDebug(true)  -- uncomment for debug mode
torch.setdefaulttensortype('torch.FloatTensor')
torch.manualSeed(321)

function pretrained_forward(rgbIm, featIm)
  -- GLOBALS: network0, networkd
  network0:evaluate()
  networkd:evaluate()

  local input = network0:forward(rgbIm)	-- process RGB       
  local inputd2 = networkd:forward(featIm):squeeze()   -- HHA 
  local input_concat = torch.cat(input,inputd2,1):squeeze()  -- concat RGB,  HHA
  -- free memory for hires
  network0:clearState()
  networkd:clearState()
  return input_concat
end  

function test_predictor(rgbIm, featIm, dc)
  -- GLOBALS: network
  pm('Segmenting...')
  local input_concat = pretrained_forward(rgbIm, featIm)
--   if params.densecap ~= 0 then
--     input_concat = {input_concat, dc}
--   end

  local output = network:forward(input_concat) -- final prediction
--   if params.densecap ~= 0 then output = output[1] end
  pm('... done')
  return output:exp() -- NOTE: output (log-space) is converted to probabilities
end



function initAPCSegmentation()
  
  createAuxDirs()
  -- GLOBAL INFO AND NETWORKS
  classes, classes_names = getClassInfo()
  dtype, use_cudnn = setup_gpus(params.cuda_device, 1)
  loadPretrainedModels()
  loadSegmentationModel()
end


--------------------------------------------------------------------------
--- run segmentation on one image
-- @param	rgbIm	3 channel (RGB) image
-- @param	featIm	depth (1 channel) or HHA (3 channels) image
-- @param	binMsak	estimated region of interest as a binary image
function run_image(rgbIm, featIm, binMask, dc)
  -- Reset the cuda device since some other method (e.g. densecap) might have
  -- changed it.
  cutorch.setDevice(segmentation_cudaDevice)

  featIm[featIm:ne(featIm)]=0 -- remove nans
--   printImageStats(featIm)
  
  pm('Preprocessing image ', 2)
  local crpIm, crpHHA, crpInfo = processInput(rgbIm, featIm, binMask, dc) -- 'crop region of interest'
  
  local output = test_predictor(crpIm, crpHHA):float()	-- run CNN
  output = vecToTensor(output) -- convert prediction to image-sized tensor with #classes channels
  local outputFull = unwarpTensor(output, crpInfo) -- 'uncrop'
--   local segMask = outputToMask(output)			-- get indexed mask
--   local segMaskFull = unwarpImage(segMask, crpInfo)	-- 'uncrop' mask
  local segMaskFull = torch.ByteTensor(outputFull:size(2), outputFull:size(3))

  network:clearState()

  -- return indexed mask and raw probabilities
  local out = {
    mask = segMaskFull,
    prediction = outputFull
  }
  return out
end


