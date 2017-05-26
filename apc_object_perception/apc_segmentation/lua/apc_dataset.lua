--require "mattorch"
require "torch"
require "math"
require "image"
torch.setdefaulttensortype('torch.DoubleTensor')
matio = require 'matio'

local train_data, test_data = nil, nil
local hha_train, hha_test = nil, nil

--------------------------------------------------------------------------
--- Print size of a tensor
function printDataSize(data)
  local sizeVec = data:size()
  local nDim = data:nDimension()
  println = ''
  for s=1,nDim do
    println = println..' '..sizeVec[s]
    if s<nDim then println = println..' x' end
  end
  print(println)
end

--------------------------------------------------------------------------
--- Show label class ratio within a dataset
function printLabelDistribution(data, nclasses)
  -- label distribution
  local nPix = data:nElement()
  -- include 0 and N+1
  local allLabs = torch.cat(torch.Tensor({0}),torch.cat(torch.linspace(1,nclasses,nclasses),torch.Tensor({nclasses+1})))
  local ratios = torch.zeros(nclasses)
  local prline = 'Label: '
  local prliner = 'Ratio: '
  for l=1,nclasses+2 do
    local ratio = torch.sum(data:eq(allLabs[l])) / nPix
    prline = prline .. string.format('%5d',allLabs[l])
    prliner = prliner .. string.format('%5.1f', 100 * ratio)
    if l%10==0 then prline = prline .. '\n' prliner = prliner .. '\n' end
    if l>1 and l<=nclasses+1 then ratios[l-1] = ratio end
  end
  print(prline)
  print(prliner)
  return ratios
end

-- DEPRECATED
-- function remapLabels(data)
--   if params.obj_class > 0 then
--     -- map everything to class / non-class
--     for o=1,40 do 
--       if o~= params.obj_class then
-- 	data[data:eq(o)]=200
--       else
-- 	data[data:eq(o)]=100
--       end
--     end
--     data[data:eq(200)]=2
--     data[data:eq(100)]=1
--     data[data:eq(41)]=3
--   else
--     -- map everything to object / box / unknown
--     for o=3,40 do data[data:eq(o)]=1 end
--     data[data:eq(41)]=3 --  unknown to 3
--   end
--   return data
-- end

function keepOneClass(data, hha)
  if params.obj_class == 0 then return data, hha end
  
  local nfiles = data.imArray:size(1)  
  local idx = {}
  for t=1,nfiles do
    if torch.sum(data.labelArray[{t,{},{}}]:eq(params.obj_class))>0 then
      table.insert(idx,t)
    end
  end
  
  if #idx==0 then 
    pm('WARNING. No positive samples found. Keep first dataset instance only',1)
    table.insert(idx, 1) 
  end
  idx = torch.Tensor(idx):long()
  data.imArray = data.imArray:index(1,idx)
  data.labelArray  = data.labelArray:index(1,idx)
  hha.hha = hha.hha:index(1,idx)
  
  return data, hha
end

function resizeAll(data)
  local nfiles = data:size(1)
  local imArray = torch.FloatTensor(nfiles,data:size(2),480,640)
  for t = 1, nfiles do
    imArray[{t,{},{},{}}] = image.scale(data[{t,{},{},{}}],480,640)
  end
  return imArray

end

function read_dc_chunk(fileName, varname)
    local data = {}
    pm('Loading densecap...')
    data.dcprobs = matio.load(fileName, varname):permute(4,3,2,1)
    pm('Fixing nans...')
    data.dcprobs[data.dcprobs:ne(data.dcprobs)]=0 -- replace nans with 0
    
    local nF, nC, iH, iW = data.dcprobs:size(1), data.dcprobs:size(2), data.dcprobs:size(3), data.dcprobs:size(4)
    local ds = params.downsample
--     local ds = 1
    local iHDS, iWDS = iH/ds, iW/ds
    pm(string.format('Files: %d\tChannels: %d\t HxW: %dx%d',nF,nC,iH,iW))
    
    local newDC = torch.zeros(nF, nC, iHDS, iWDS)
    pm('Preparing dcprobs...')
    for t = 1, newDC:size(1) do
      for c=1,newDC:size(2) do 
	local s = torch.sum(data.dcprobs[{{t},{c},{},{}}])
	if s>0 then
	  newDC[{{t},{c},{},{}}] = 
	    image.scale(data.dcprobs[{{t},{c},{},{}}]:squeeze(), iWDS, iHDS,  'simple')
-- 	  wdcp = showImage(data.dcprobs[{{t},{c},{},{}}]:reshape(1,iH,iW), wdcp)
-- 	  print(t,c)	      
-- 	  wdc = showImage(newDC:narrow(1,t,1):narrow(2,c,1):squeeze(), wdc) sleep(.11)
	else
	  newDC[{{t},{c},{},{}}] = torch.zeros(iWDS, iHDS)
	end
      
      end
    end
--       data.dcprobs = newDC:permute(1,2,4,3) -- FIXME REMOVE PERMUTE WHEN DATASET FIXED
    data.dcprobs = newDC
    
    
    for t = 1, newDC:size(1) do
      for c=1,newDC:size(2) do 
	local s = torch.sum(newDC[{{t},{c},{},{}}])
	if s>0 then
-- 	    print(torch.max(newDC[{{t},{c},{},{}}]))
	  newDC[{{t},{c},{},{}}] = newDC[{{t},{c},{},{}}]/torch.max(newDC[{{t},{c},{},{}}])
-- 	    sleep(1)
-- 	    wdc = showImage(newDC:narrow(1,t,1):narrow(2,c,1):squeeze(), wdc) sleep(.1)
	end
      end
    end
    

    return newDC
end

function load_data(fileName, featType)
--   featType = featType or false
  print('Loading ' ..fileName ..'...')

--   print(featType)
  local data = {}
  if featType ~= nil then
    
    if featType == 'depths' then
      data.hha = matio.load(fileName, 'depths')
      data.hha=data.hha:permute(3,2,1)
      data.hha[data.hha:ne(data.hha)]=0 -- replace nans with 0
      
      -- expand 1d to 3d
      local newFeatures = torch.zeros(data.hha:size(1), 3, data.hha:size(2), data.hha:size(3))
      for t = 1, data.hha:size(1) do
	newFeatures[{{t},{},{},{}}] = prepDepth(data.hha[{{t},{},{}}])	
      end
      data.hha = newFeatures
    elseif featType == 'hha' then
      data.hha = matio.load(fileName, 'hha')
      data.hha = data.hha:permute(4,3,2,1)
      data.hha[data.hha:ne(data.hha)]=0 -- replace nans with 0
    elseif featType == 'rha' then
      data.hha = matio.load(fileName, 'hha')
      data.hha = data.hha:permute(4,3,2,1)
      
      local relMeanDiff = matio.load(fileName, 'rmd')
      relMeanDiff = relMeanDiff:permute(3,2,1)
      data.hha[{{},{1},{},{}}] = relMeanDiff
      
--       wdc = showImage(data.hha:narrow(1,1,1):narrow(2,2,1):squeeze(), wdc) sleep(.11)
      
      data.hha[data.hha:ne(data.hha)]=0 -- replace nans with 0
    else 
      error('unknown feature type')
    end
    
    if params.cuda_device > 0 then data.hha = data.hha:cuda() end
  else
    data.imArray = matio.load(fileName, 'imArray'):permute(4,3,2,1)
    data.labelArray = matio.load(fileName, 'labelArray'):permute(3,2,1)
    print(data.labelArray:size())
    if params.cuda_device > 0 then 	
      data.imArray = data.imArray:cuda()
      data.labelArray = data.labelArray:cuda()
--       if params.num_classes == 6 then
-- 	-- do some mask picking	
-- 	local boxMasks = data.labelArray:eq(2)
-- 	local bonesMasks = data.labelArray:eq(1)
-- 	local objMasks = (data.labelArray:ge(3) and data.labelArray:le(40))
-- 	local shelfMasks = data.labelArray:ge(41)
-- 	data.labelArray = boxMasks:cat(objMasks,2):cat(shelfMasks,2)
-- 	print(data.labelArray:size())
-- 	
-- -- 	  data.labelArray = data.labelArray:index(1,torch.LongTensor(
--       end
    end
    if params.densecap ~= 0 then
      local dp1 = read_dc_chunk(fileName, 'dcprobs1')
      local dp2 = read_dc_chunk(fileName, 'dcprobs2')
      local dp3 = read_dc_chunk(fileName, 'dcprobs3')
      local dp4 = read_dc_chunk(fileName, 'dcprobs4')
      
      data.dcprobs = dp1:cat(dp2,2):cat(dp3,2):cat(dp4,2)
      data.dcprobs = data.dcprobs:cuda()
      printDataSize(data.dcprobs)
      
      data.dcprobs:narrow(2,2,1):fill(1)
      data.dcprobs:narrow(2,params.num_classes,1):fill(1)
    end    
    
  end
  


  --  print(data)
  --  os.exit()
  --  data.labelArray


  return data
end

function downsample_data(data, ds)
  local nfiles = data.imArray:size(1)
  imArray = torch.FloatTensor(nfiles,3,480/ds,640/ds)
  for t = 1, nfiles do
    imArray[{t,{},{},{}}] = image.scale(data.imArray[{t,{},{},{}}],480/ds,640/ds)
  end
  return imArray
end

function normalizeData(trainData, testData)
  local nChannels = trainData:size(2)
  mean = torch.zeros(nChannels) -- store the mean, to normalize the test set in the future
  stdv  = torch.ones(nChannels) -- store the standard-deviation for the future
  for i=1,nChannels do -- over each image channel
      mean[i] = trainData[{ {}, {i}, {}, {}  }]:mean() -- mean estimation
      stdv[i] = trainData[{ {}, {i}, {}, {}  }]:std() -- std estimation
      
      print(string.format('Channel %d: Mean %.2f,  Std: %.2f', i, mean[i], stdv[i]))
      trainData[{ {}, {i}, {}, {}  }]:add(-mean[i]) -- mean subtraction
      testData[{ {}, {i}, {}, {}  }]:add(-mean[i]) -- mean subtraction      
      
      trainData[{ {}, {i}, {}, {}  }]:div(stdv[i]) -- std scaling
      testData[{ {}, {i}, {}, {}  }]:div(stdv[i]) -- std scaling
  end
  
  local normInfo = mean:cat(stdv,2)
  
  return trainData, testData, normInfo
end


-- local suffix = '_test'
-- local suffix = '_light'
local suffix = ''
if params.data_suffix ~= '' then 
  suffix = '_'..params.data_suffix
end
-- local split = 'train'torch.max(newDC[{{t},{c},{},{}}])
-- local split = 'all'
local split = params.train_split

if params.num_classes==3 and params.obj_class>0 then suffix='' end
local keepSamples = 500 -- how many samples to keep from data

local traininDataFile = string.format('%s/%s_%s_%s%s.mat',
  params.data_path,
  params.data_file,
  params.setting,
  split,
  suffix)
  print(traininDataFile)
--   abort()
local testDataFile= string.format('%s/%s_%s_test%s.mat',
  params.data_path,
  params.data_file,
  params.setting,
  suffix)

  
hha_train = load_data(traininDataFile, params.feature_type)
hha_test = load_data(testDataFile, params.feature_type)

train_data = load_data(traininDataFile)
if train_data.imArray:size(1)>keepSamples then train_data.imArray =train_data.imArray:narrow(1,1,keepSamples) end
train_data, hha_train = keepOneClass(train_data, hha_train)


test_data = load_data(testDataFile)
if test_data.imArray:size(1)>keepSamples then test_data.imArray =test_data.imArray:narrow(1,1,keepSamples) end
test_data, hha_test = keepOneClass(test_data, hha_test)

-- print('*** WAAAARNINGGG!!!1! ***')
-- train_data.labelArray[train_data.labelArray:eq(2)] = 41
-- train_data.labelArray[train_data.labelArray:eq(41)]:fill(0)

print('Training images and labels')
printDataSize(train_data.imArray)
printDataSize(train_data.labelArray)
printDataSize(hha_train.hha)
print('Testing images, labels, features')
printDataSize(test_data.imArray)
printDataSize(test_data.labelArray)
printDataSize(hha_test.hha)

params.imHeight, params.imWidth = train_data.imArray:size(3), train_data.imArray:size(4)

if params.num_classes == 2 or params.num_classes == 3 then
  test_data.labelArray = remapLabels(test_data.labelArray)
  train_data.labelArray = remapLabels(train_data.labelArray)
end

print('Train label distribution')
train_ratios = printLabelDistribution(train_data.labelArray,params.num_classes)
print('Test label distribution')
test_ratios = printLabelDistribution(test_data.labelArray,params.num_classes)


-- train_data, test_data, data_norm = normalize_data(train_data, test_data)
-- hha_train, hha_test, feat_norm = normalize_features(hha_train, hha_test)
pm('Normalizing data...')
train_data.imArray, test_data.imArray, data_norm = normalizeData(train_data.imArray, test_data.imArray)
hha_train.hha, hha_test.hha, feat_norm = normalizeData(hha_train.hha, hha_test.hha)
local normInfo = {normDataInfo = data_norm, normFeaturesInfo = feat_norm}

infoFilename = getNormInfoFilename()
pm('Saving normalization info to '..infoFilename .. '...')
torch.save(infoFilename, normInfo)

paramFilename = getParamsFilename()
pm('Saving parameters into '..paramFilename .. '...')
torch.save(paramFilename, params)


print('Data loaded...')


-- throw data clone to another GPU
if params.mode == 'train' and params.cuda_device > 0 then
  pm('Cloning data to another GPU...')
  cutorch.setDevice(params.cuda_device_train)
  train_data_clone = {imArray = train_data.imArray:clone():cuda(), labelArray = train_data.labelArray:clone():cuda()}
  test_data_clone = {imArray = test_data.imArray:clone():cuda(), labelArray = test_data.labelArray:clone():cuda()}  
  hha_train_clone = {hha = hha_train.hha:clone():cuda()}
  hha_test_clone = {hha = hha_test.hha:clone():cuda()}
  pm(string.format('%20s%6s%6s','Variable','Orig','Clone'),2)
  pm(string.format('%20s%6d%6d','train img',train_data.imArray:getDevice(), train_data_clone.imArray:getDevice()),2)
  pm(string.format('%20s%6d%6d','train lab',train_data.labelArray:getDevice(), train_data_clone.labelArray:getDevice()),2)
  pm(string.format('%20s%6d%6d','train feat',hha_train.hha:getDevice(), hha_train_clone.hha:getDevice()),2)
  pm(string.format('%20s%6d%6d','test img',test_data.imArray:getDevice(), test_data_clone.imArray:getDevice()),2)
  pm(string.format('%20s%6d%6d','test lab',test_data.labelArray:getDevice(), test_data_clone.labelArray:getDevice()),2)
  pm(string.format('%20s%6d%6d','test feat',hha_test.hha:getDevice(), hha_test_clone.hha:getDevice()),2)
  
  
  cutorch.setDevice(params.cuda_device)
end

return train_data, test_data, hha_train, hha_test

