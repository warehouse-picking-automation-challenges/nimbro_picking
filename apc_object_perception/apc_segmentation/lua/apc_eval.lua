require 'apc'		-- For test code
 
params = {}

setParams()

-- params.num_classes = 44
local allClasses, allClassesNames = getClassInfo()
local allMetrics = {}
for k,v in pairs(allClassesNames) do
  allMetrics[v] = {p={}, r={}} -- precision, recall for this class
end  
  


initAPCSegmentation()
loadSegmentationModel()

-- print(getDatasetDir()..'/'..params.data_file..'_'..params.setting..'_test_files.txt')
-- abort()

local files = {}
for line in io.lines(getDatasetDir()..'/'..params.data_file..'_'..params.setting..'_test_files.txt') do
  files[#files + 1] = line
end

for f = 1,#files do
-- for f = 3,3 do
  params.im_dir = getDatasetDir()..files[f] .. '/'
  local rgbIm, depthIm, boxMask = readImage(params.im_dir)
  local instDir = getProjectRoot() .. '/predictions/'..files[f]      
  if not lfs.attributes(instDir) then os.execute(string.format('mkdir -p %s',instDir)) end

  pm('Reading image from disk '..params.im_dir, 2)
  local rgbIm, depthIm, boxMask, dc = readImage(params.im_dir)
  local crpIm, crpHHA, crpInfo, dc = processInput(rgbIm, depthIm, boxMask, dc)
  dc = dc:float()

  local trimClasses = getRelevantClasses(allClassesNames)
--   trimClasses = trimClasses:sub(1,-2) -- no need for unknown
  
  ----- 1 VS ALL MODE
  if params.num_classes == 3 then 
    for c=1,trimClasses:nElement() do
      local currentClassID = trimClasses[c]
      local className = allClassesNames[currentClassID]
--       print(className)
      params.obj_class = currentClassID
      local gtMask = image.load(getDatasetDir()..files[f]..'/mask_'..className..'.png')
      
      loadSegmentationModel() -- 1 vs all
      
      local output = test_predictor(crpIm, crpHHA, dc)      
      local segMask = outputToMask(output):float()

      segMask = outputToMask(output):float()
      segMask = unwarpImage(segMask, crpInfo.transformInfo, crpInfo):float()
      segMask[segMask:ne(1)] = 0
      
      local precision, recall = evalSegmentation(gtMask, segMask)
      table.insert(allMetrics[className].p,precision)
      table.insert(allMetrics[className].r,recall)
    end
      
  elseif params.num_classes >= 41 then ----- ALL VS ALL MODE
    loadSegmentationModel() -- all vs all
--     print(crpIm:size())
--     print(crpHHA:size())
--     print(dc:size())
    local output = test_predictor(crpIm, crpHHA, dc):float()
    output = vecToTensor(output)      




    for xi=1,dc:size(1) do
      output[xi]:cmul(dc[xi])
    end

    local outputFull = unwarpTensor(output, crpInfo)

    local segMask = outputToMask(output, trimClasses)
    local segMaskFull = outputToMask(outputFull, trimClasses)
--     smf = showImage(segMaskFull, smf)
--     sleep(5)

    
    for c=1,trimClasses:nElement() do
      local classMask = segMaskFull:clone()
      local currentClassID = trimClasses[c]
      local className = allClassesNames[currentClassID]
      
      if className ~= 'unknown' then
	local gtMask = image.load(getDatasetDir()..files[f]..'/mask_'..className..'.png')
	classMask[classMask:ne(c)] = 0
	classMask[classMask:eq(c)] = 1
	
  --       showImage(classMask, sm)
  --       showImage(gtMask, gm)
  --       sleep(5)
	
	
	local classPrediction = outputFull:narrow(1,currentClassID,1)
	-- write out binary mask and raw predictions
	
	
	image.save(instDir..'/segmentation_'..className..'.png', classMask)
	image.save(instDir..'/prediction_'..className..'.png', classPrediction)      
	
	local precision, recall, f1 = evalSegmentation(gtMask, classMask)
	pm(string.format('%-35s Prec: %6.4f \t Recl: %6.4f \t F1: %6.4f',className..'...', precision, recall, f1))
	table.insert(allMetrics[className].p,precision)
	table.insert(allMetrics[className].r,recall)      
      end
    end
    image.save(instDir..'/rgb.png', rgbIm)
    ci = showImage(crpIm, ci)
    sm = showImage(segMask, sm)
    
  end

end


-- wipe eval file
logFilename = getProjectRoot() .. '/predictions/eval_'..params.model_name..'.txt'
if params.epochs~=nil and params.epochs>0 then
  logFilename = getProjectRoot() .. '/predictions/eval_'..params.model_name..'_ep'..params.epochs..'.txt'
end
local file = io.open(logFilename, 'w')
file:close()

  
for currentClassID=1,params.num_classes-1 do
  local className = allClassesNames[currentClassID]  
  local prec, rec = allMetrics[className].p, allMetrics[className].r
--   local logFilename = getProjectRoot()..'/predictions/eval_'..className..'.txt'    
--   file = io.open(logFilename, 'w')
--   for inst = 1,#prec do
--     file:write(string.format('%f,%f\n',prec[inst], rec[inst]))
--   end  
--   file:close()
  
  -- print summary
  
  local p,r,f1= 0,0,0
  if #prec>0 then
    p = torch.Tensor(prec):mean()
    r = torch.Tensor(rec):mean()
    f1 = 2 * (p*r) / (p+r)
    if p+r == 0 then f1 = 0 end
  end
  local oneLine = string.format('%-40s Precision : %.3f   Recall : %.3f   F1  : %.3f',
    className,p,r,f1)
    
  print(oneLine)
  
  logFilename = getProjectRoot() .. '/predictions/eval_'..params.model_name..'.txt'
  file = io.open(logFilename, 'a')
  file:write(oneLine..'\n')
  file:close()
end
os.exit()

-- print(files)
