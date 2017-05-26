require 'apc'		-- For test code
require 'cunn'


ggtime = torch.Timer()
profTable = {}

--------------------------------------------------------------------------
--- Updates a global profiling table
function profUpdate(fname, ftime)
  if profTable[fname] == nil then
    profTable[fname] = torch.Tensor({0,0})
  end
  profTable[fname] = profTable[fname] + torch.Tensor({ftime, 1})
end



local loctimer=torch.Timer()
setParams()
profUpdate('setParams', loctimer:time().real)

loctimer=torch.Timer()
initAPCSegmentation()
profUpdate('init', loctimer:time().real)

loctimer=torch.Timer()
loadSegmentationModel()
profUpdate('loadModel', loctimer:time().real)
-- print(classes_names)



-- local imW, imH = 1920,1080
-- local imW, imH = 960,540
-- local imW, imH = 640,480
-- local imW, imH = 768,576
--- REPLACE
-- network:remove(9)
-- network:insert(nn.Reshape(imW*imH,params.num_classes):cuda(),9)

local trimClasses = nil
if params.num_classes >= 41 then 
  trimClasses = getRelevantClasses()
end
-- local trimClasses = nil
-- print(trimClasses)
-- trimClasses = torch.Tensor({24, 8, 3, 10, 16, 36, 2, 41,42,43,44})
-- trimClasses = torch.Tensor({1, 2, 15, 33, 34, 38, 26, 24, 41})
-- trimClasses = setObjectList(
--   {'kyjen_squeakin_eggs_plush_puppies',
--     'hanes_tube_socks',
--     'scotch_duct_tape',
--     'box','unknown'})
-- trimClasses = setObjectList(
--   {
-- 'box',
-- 'command_hooks',
-- 'crayola_24_ct',
-- 'dr_browns_bottle_brush',
-- 'elmers_washable_no_run_school_glue',
-- 'expo_dry_erase_board_eraser',
-- 'fiskars_scissors_red',
-- 'fitness_gear_3lb_dumbbell',
-- 'scotch_bubble_mailer',
-- 'soft_white_lightbulb',
-- 'staples_index_cards',
-- 'womens_knit_gloves',
-- 'unknown'
--   }
--   )

-- trimClasses = setObjectList(
--   {
--     'box',
--     'barkely_hide_bones',		-- 1
--     'dove_beauty_bar',			-- 11
--     'dr_browns_bottle_brush',           -- 12
--     'easter_turtle_sippy_cup',          -- 13
--     'elmers_washable_no_run_school_glue',-- 14
--     'expo_dry_erase_board_eraser',      -- 15
--     'kyjen_squeakin_eggs_plush_puppies',-- 24
--     'peva_shower_curtain_liner',        -- 28
--     'rolodex_jumbo_pencil_cup',         -- 31
--     'scotch_duct_tape',                 -- 34
--     'staples_index_cards',              -- 36
--     'ticonderoga_12_pencils',           -- 37
--     'unknown'             		-- 41
--   }
--   )

-- print(trimClasses)


pm('Reading image from disk '..params.im_dir, 2)

loctimer=torch.Timer()
local rgbIm, featIm, boxMask, dc = readImage(params.im_dir)
profUpdate('readImage', loctimer:time().real)

-- showImage(rgbIm, rgb)

-- local segMask = run_image(rgbIm, featIm, boxMask) 
-- segMask = segMask.mask


pm('Preprocessing image ', 2)
loctimer=torch.Timer()
local crpIm, crpHHA, crpInfo, dc = processInput(rgbIm, featIm, boxMask, dc)
profUpdate('processInput', loctimer:time().real)

showImage(crpIm, win_input)
showImage(crpHHA, win_feat)
if params.densecap ~= 0 or params.dc_setting == 3 then 
  win_dc = showImage(dc:narrow(1,1,1), win_dc) 
end
-- sleep(1)
-- print(crpIm:type())
-- printImageStats(crpIm)
-- crpIm = image.scale(crpIm:float(), imW,imH):cuda()
-- crpHHA = image.scale(crpHHA:float(), imW,imH):cuda()
loctimer=torch.Timer()
-- local output = test_predictor(crpIm, crpHHA):float()

profUpdate('testing', loctimer:time().real)
-- print(dc:size())
local output = test_predictor(crpIm, crpHHA, dc):float()

-- local input_concat = pretrained_forward(crpIm, crpHHA)
-- local input = network0:forward(crpIm)	-- process RGB       
-- local inputd2 = networkd:forward(crpHHA):squeeze()   -- HHA 
-- local input_concat = torch.cat(input,inputd2,1):squeeze()  -- concat RGB,  HHA
-- 
-- if params.densecap ~= 0 then
--   input_concat = {input_concat, dc:cuda()}
-- end

-- local output = network:forward(input_concat):float():exp()

-- print(output:size())

loctimer=torch.Timer()
output = vecToTensor(output)
profUpdate('vecToTensor', loctimer:time().real)
loctimer=torch.Timer()

if params.dc_setting == 3 then
  dc:narrow(1,2,1):fill(1)
  dc:narrow(1,params.num_classes,1):fill(1)
  output = torch.cmul(output, dc)
--   for c=1,params.num_classes do 
--     output[{{c},{},{}}] = torch.cmul(output[{{c},{},{}}], dc[{{c},{},{}}])
--   end
end
-- print(output:size())
local outputFull = unwarpTensor(output, crpInfo)
profUpdate('unwarp', loctimer:time().real)

-- local trimClasses = torch.Tensor({2,26,31}):long()
loctimer=torch.Timer()
local segMask = outputToMask(output, trimClasses)
local segMaskFull = outputToMask(outputFull, trimClasses)
profUpdate('toMask', loctimer:time().real)

local colLabels = colorizeLabels(segMaskFull)
local alpha = 0.8
local overlayIm = (rgbIm:float() * (1-alpha)) + (colLabels * (alpha))


image.save(getProjectRoot()..'/predictions/overlay.png',overlayIm)
image.save(getProjectRoot()..'/predictions/seg_colored.png',colLabels)


if params.num_classes >= 41 and trimClasses ~= nil then
  colLabels = drawCaptions(colLabels, trimClasses)
end
image.save(getProjectRoot()..'/predictions/seg_colored_captioned.png',colLabels)

-- showImage(segMask, win_pl)
showImage(overlayIm, win_ol)
showImage(colLabels, win_cl)
-- showImage(segMaskFull, win_pl_f)
-- sleep(10)


if trimClasses ~= nil then
  for c=1,trimClasses:nElement() do
    local classID = trimClasses[c]
    local classProb = output:narrow(1,classID,1):squeeze()
    image.save(getProjectRoot()..'/predictions/predictions_'..classes_names[classID]..'.png', classProb)
    cp = showImage(classProb, cp)
    
    
    -- print evaluation
    local className = classes_names[classID]
    gtMaskFile = params.im_dir..'/mask_'..className..'.png'
    gtExists = lfs.attributes(gtMaskFile)
    
    if className ~= 'unknown' and gtExists then
      local classMask = segMaskFull:clone()
      classMask[classMask:ne(c)] = 0
      classMask[classMask:eq(c)] = 1      

      
      
      local gtMask = image.load(gtMaskFile)
      local precision, recall, f1 = evalSegmentation(gtMask, classMask)
      pm(string.format('%-35s Prec: %6.4f \t Recl: %6.4f \t F1: %6.4f',className..'...', precision, recall, f1))
    end
    
    sleep(.1)
  end
end


-- printImageStats(segMask)

loctimer=torch.Timer()
image.save(getProjectRoot()..'/predictions/rgb.png', rgbIm)
image.save(getProjectRoot()..'/predictions/featIm.png', featIm)
image.save(getProjectRoot()..'/predictions/mask_box.png', boxMask)
image.save(getProjectRoot()..'/predictions/segmentation.png', segMask/segMask:max())
image.save(getProjectRoot()..'/predictions/segmentation_full.png', outputFull:narrow(1,2,1):squeeze())

local segMask = outputToMask(output):byte()
-- for c=1,#classes do
--   local classMask = torch.zeros(segMask:size())
--   classMask[segMask:eq(c)] = 1
--   image.save(getProjectRoot()..string.format('/predictions/segmentation_%s.png',classes_names[c]), classMask)
--   
-- end
profUpdate('saving png', loctimer:time().real)

-- local output = run_image(im, depth, binMask)

local mtime = 0
print('-------------   PROFILING   INFO   ----------------')
print(string.format('%20s%10s%7s','function name','time','calls'))
for k,v in pairs(profTable) do
  if v[1] > 0.1 then
    print(string.format('%20s%10.2f%7d',k,v[1],v[2]))
  end
  mtime = mtime + v[1]
end
print(string.format('%20s%10.2f%7s','total time meas.',mtime,''))
print(string.format('%20s%10.2f%7s','total time',ggtime:time().real,''))

