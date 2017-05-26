-----------------------
--- Module including many miscellaneous utility functions
--
-- @module types


--------------------------------------------------------------------------
--- Sleep for n seconds.
-- @param n Time to hold in seconds.
function sleep(n)  -- seconds
  local clock = os.clock
  local t0 = clock()
  while clock() - t0 <= n do end
end

--------------------------------------------------------------------------
--- Abort the execution of the program (for debugging)
-- @param msg 	optional abort message
function abort(msg) 
  msg = msg or ''
  print("Aborting "..msg)
  os.exit(0)
end



--------------------------------------------------------------------------
--- Make directory if does not yet exist
-- @param dir   path to dir to create
function mkdirP(dir)
  if not lfs.attributes(dir) then 
    lfs.mkdir(dir)
    pm(string.format('Directory %s created',dir)) 
  end
end

-- see if the file exists
function fileExists(file)
  local f = io.open(file, "rb")
  if f then f:close() end
  return f ~= nil
end


function getProjectRoot()
  return params.packagePath
end

function getDatasetDir()
    local datasetDir = '../apc_data'
    datasetDir = '/home/milan/research/projects/APC/apc_data/'
    if onBigcuda() then 
      datasetDir = '/home/local/staff/milan/research/projects/APC/apc_data/' 
    end
    datasetDir = '/home/data/stud/periyasa/apc_data/'
    return datasetDir
end


--------------------------------------------------------------------------
--- Make auxiliary directories quietly
function createAuxDirs()
  local rootDir = getProjectRoot()
  mkdirP(rootDir..'/models')
  mkdirP(rootDir..'/predictions')
end

--------------------------------------------------------------------------
--- Test whether we are running on bigcuda with strong GPU
function onBigcuda()
  if lfs.attributes('/home/local/staff/milan/','mode') then
    return true
  end

  return false
end



--------------------------------------------------------------------------
--- Prints depending on the debug level set in opt.verbose
-- @param message   The message to be printed
-- @param vl    The verbosity level (0=none, 1=warning, 2=info, 3=debug)
function pm(message, vl)
  vl=vl or 2
  local pv = 2
  if params ~= nil and params.verbose ~= nil then pv=params.verbose end
  if vl <= pv then
    print(message)
  end
end

--------------------------------------------------------------------------
--- Are we running from command line or from luaC
function runFromCmdLine() 
  if arg~=nil then return true end
  return false  
end

--------------------------------------------------------------------------
--- Show image in qlua
function showImage(img, win_descr)
  if not runFromCmdLine() then return end -- ignore if running from luaC
  if params.suppress_x == 1 then return end -- ignore if vis suppressed
  
  local disp_win = image.display{image=img, win=win_descr, zoom=1}
  return disp_win
end



function getIDColors()
  local colors=torch.Tensor({
      {128,255,255},    -- 
      {255,0,0},        -- red           1
      {0, 255, 0},        -- green         2
      {0, 0, 255},        -- blue          3
      {0, 255, 255},      -- cyan          4
      {255, 0, 255},      -- magenta       5
      {212, 212, 0},      -- yellow        6
      {25, 25, 25},       -- black         7
      {34,139,34},      -- forestgreen   8
      {0,191,255},      -- deepskyblue   9
      {139,0,0 },       -- darkred       10
      {218,112,214},    -- orchid        11
      {244,164,96}, -- sandybrown    12
      {245,245,245}, -- white smoke      13
      {139,119,101},    -- peach puff 4         14
      {105,105,105},    -- dim gray     15
      {25,25,112},      -- midnight blue 16
      {70,130,180},     -- steel blue   17
      {64,224,208},     -- turqoise     18
      {95,158,160},     -- cadet blue   19
      {106,19,205},     -- slate blue   20
      {102,205,170},     -- Medium Aquamarine       21
      {152,251,152},     -- Pale Green   22
      {240,230,140},     -- Khaki   23
      {255,215,0},     -- Gold   24
      {184,134,11},     -- Dark Goldenrod   25      
      {188,143,143},     -- Rosy Brown   26
      {160,82,45},     -- Sienna   27
      {245,245,220},     -- Beige   28
      {210,180,140},     --  Tan  29
      {178,34,34},     -- Firebrick   30
      {233,150,122},     -- Dark Salmon       31
      {255,165,0},     --  Orange  32
      {255,99,71},     -- Tomato   33
      {240,128,128},     -- Light Coral       34
      {255,105,180},     -- Hot Pink         35      
      {255,192,203},     --  Pink  36
      {221,160,221},     -- Plum   37
      {245,222,179},     -- Wheat     38
      {255,250,205},     --  Lemon Chiffon   39
      {160,32,240},     --  Purple  40
      {210,105,30},     --  Chocolate  41
      {127,255,0},     --  Chartreuse      42
      {153,50,204},     --  Dark Orchid    43
      {216,191,216}      -- Thistle   44      
  })  
  colors = colors:div(255);

  return colors
end

function getColorFromID(id)
-- get rgb [0,1] values from id

  local col = torch.zeros(3)
  if id==0 then return col end
  colors=getIDColors();
  
--   print(colors)
  col=colors[(id % colors:size(1))+1];
  return col
end

--------------------------------------------------------------------------
--- Replace grayscale label image with colored one
-- DEPRECATED
-- function colorizeLabelsOLD(labelIm)
--   if not runFromCmdLine() then return labelIm end
--   local colLabelIm = torch.zeros(3,labelIm:size(1), labelIm:size(2)):float()
--   for o=1,params.num_classes do
--     oMask = labelIm:eq(o)
--     if oMask:sum()>0 then
--       local col = getColorFromID(o)
--       for c=1,3 do
-- 	local colMask = torch.ones(labelIm:size()):float()*col[c]
-- 	colLabelIm[{{c},{},{}}] = colLabelIm[{{c},{},{}}]:add(oMask:float():cmul(colMask))
--       end
--     end
--   end
--   return colLabelIm
-- end

--------------------------------------------------------------------------
--- Replace grayscale label image with colored one
function colorizeLabels(labelIm)

  local colLabelIm = torch.zeros(3,labelIm:size(1), labelIm:size(2)):float()
  
  for o=1,params.num_classes do
    oMask = labelIm:eq(o)
    if oMask:sum()>0 then
      local col = getColorFromID(o)
      for c=1,3 do
--       for c=1,1 do
        colLabelIm:narrow(1,c,1):maskedFill(labelIm:eq(o), col[c])
      end
    end
  end
  
  return colLabelIm
end

--------------------------------------------------------------------------
--- Directory where network models are stored
function getModelsPath()
  return params.packagePath .. '/models/'
end

--------------------------------------------------------------------------
--- A setting-specific string
function getModelSignature(epoch)
  
  local modelSignature = 
    "_ds"..params.downsample..
    "_cl"..#classes..
    "_obj"..params.obj_class
--     epoch = 10
    
    if epoch ~= nil and epoch ~= 0 then modelSignature = modelSignature .. "_ep" .. epoch end
    
    modelSignature = modelSignature ..
      "_"..params.setting
      
  return modelSignature
end

--------------------------------------------------------------------------
--- Main model name (identifier)
function getModelFilename(epoch)
  local modelName = params.model_name or 'apc'
  
  local modelFilename = getModelsPath() .. modelName..  getModelSignature(epoch) .. '.t7'
  return modelFilename
end

function getNormInfoFilename()
  local modelName = params.model_name or 'apc'
  
  local modelFilename = getModelsPath() .. modelName..  getModelSignature() .. '_norm.t7'
  return modelFilename
end

function getParamsFilename()
  local modelName = params.model_name or 'apc'
  
  local modelFilename = getModelsPath() .. modelName..  getModelSignature() .. '_params.t7'
  return modelFilename
end


--------------------------------------------------------------------------
--- Load OverFeat Models (torch)
function loadPretrainedModels()
  local deviceString = 'gpu'
  
--   cutorch.setDevice(params.cuda_device_train)
  -- RGB network
  local modelFilename = getModelsPath()..'/mlp0_ds'..params.downsample..'_'..deviceString..'.t7'
  pm('loading '..modelFilename)  
  network0 = torch.load(modelFilename)

  -- Depth network
  local modelFilename = getModelsPath()..'/mlpd_ds'..params.downsample..'_'..deviceString..'.t7'
  pm('loading '..modelFilename)
  networkd = torch.load(modelFilename)
  

  if params.cuda_device == 0 then 
    network0 = network0:float()
    networkd = networkd:float()
    pm('converted to cpu')
  end
  
end

--------------------------------------------------------------------------
--- Load a trained model
function loadSegmentationModel()
    
  -- final segmentation network
  local networkFileName = getModelFilename(params.epochs)
  pm('Loading '..networkFileName)
  network = torch.load(networkFileName)	-- load the network
  if params.cuda_device == 0 then 
    network = network:float()
    print('converted to cpu')
  end
end

--------------------------------------------------------------------------
--- For debugging only, look at GPU usage at test/training time
function printGPUStatus(msg)
  if not runFromCmdLine() then return end
    
  if msg ~= nil then pm(msg, 3) end
  if params.verbose >= 3 then
    os.execute(string.format('nvidia-smi | grep GeForce | head -n %d | tail -n 1', params.cuda_device))
    os.execute(string.format('nvidia-smi | grep MiB | head -n %d | tail -n 1', params.cuda_device))
  end
end


--------------------------------------------------------------------------
--- set up GPU devices
function setup_gpus(gpu, use_cudnn)
  local dtype = 'torch.FloatTensor'
  local actual_use_cudnn = false
  if gpu > 0 then
    require 'cutorch'
    require 'cunn'
    cutorch.setDevice(gpu)
    segmentation_cudaDevice = gpu
    dtype = 'torch.CudaTensor'
    if use_cudnn == 1 then
      require 'cudnn'
      actual_use_cudnn = true
    end
  end
  return dtype, actual_use_cudnn
end


--------------------------------------------------------------------------
--- Get semantic classes IDs and names
function getClassInfo()
  local classes = torch.totable(torch.linspace(1,params.num_classes, params.num_classes))
  local classes_names = {
    'barkely_hide_bones',		-- 1
    'box',				-- 2
    'cherokee_easy_tee_shirt',		-- 3
    'clorox_utility_brush',		-- 4
    'cloud_b_plush_bear',		-- 5
    'command_hooks',			-- 6
    'cool_shot_glue_sticks',		-- 7
    'crayola_24_ct',			-- 8
    'creativity_chenille_stems',	-- 9
    'dasani_water_bottle',		-- 10
    'dove_beauty_bar',			-- 11
    'dr_browns_bottle_brush',           -- 12
    'easter_turtle_sippy_cup',          -- 13
    'elmers_washable_no_run_school_glue',-- 14
    'expo_dry_erase_board_eraser',      -- 15
    'fiskars_scissors_red',             -- 16
    'fitness_gear_3lb_dumbbell',        -- 17
    'folgers_classic_roast_coffee',     -- 18
    'hanes_tube_socks',                 -- 19
    'i_am_a_bunny_book',                -- 20
    'jane_eyre_dvd',                    -- 21
    'kleenex_paper_towels',             -- 22
    'kleenex_tissue_box',               -- 23
    'kyjen_squeakin_eggs_plush_puppies',-- 24
    'laugh_out_loud_joke_book',         -- 25
    'oral_b_toothbrush_green',          -- 26
    'oral_b_toothbrush_red',            -- 27
    'peva_shower_curtain_liner',        -- 28
    'platinum_pets_dog_bowl',           -- 29
    'rawlings_baseball',                -- 30
    'rolodex_jumbo_pencil_cup',         -- 31
    'safety_first_outlet_plugs',        -- 32
    'scotch_bubble_mailer',             -- 33
    'scotch_duct_tape',                 -- 34
    'soft_white_lightbulb',             -- 35
    'staples_index_cards',              -- 36
    'ticonderoga_12_pencils',           -- 37
    'up_glucose_bottle',                -- 38
    'womens_knit_gloves',               -- 39
    'woods_extension_cord',		-- 40
    'unknown'}             		-- 41
    
  if params.num_classes == 44 then
    classes_names[41] = 'front_bar'	-- 41
    classes_names[42] = 'side_bar'	-- 42
    classes_names[43] = 'ground_metal'	-- 43
    classes_names[44] = 'unknown'	-- 44
  end
--   if params.num_classes == 6 then
--     classes_names[1] = 'box'	-- 1
--     classes_names[2] = 'object'	-- 2
--     classes_names[3] = 'front_bar'	-- 3
--     classes_names[4] = 'side_bar'	-- 4
--     classes_names[5] = 'ground_metal'	-- 5
--     classes_names[6] = 'unknown'	-- 6
--   end
  if params.num_classes == 5 then classes_names = {'floor','structure','furniture','prop', 'unknown'} end
  if params.num_classes == 3 then 
    if params.obj_class == 0 then
      classes_names = {'object', 'box','unknown'} 
    else
      classes_names = {classes_names[params.obj_class], 'clutter', 'unknown'}
      print('Segmenting '..classes_names[1])
    end
  end  
  return classes, classes_names
end

function classNameToID(className, classes)
  local c = -1
  for c=1,#classes do
    if className == classes[c] then
      return c
    end
  end
  return c
end


-- crop, scale, rotate
function prepImage(im, bxLeft, bxTop, bxWidth, bxHeight)
  local targetH, targetW = 480, 640
  if params.crop_mode == 3 then
    targetH, targetW = 540, 960
  end
--   local targetH, targetW = 576, 768
  local rotRad = math.pi
  
  im = im:narrow(2,bxTop,bxHeight):narrow(3,bxLeft,bxWidth)
--   print(bxLeft, bxTop, bxWidth, bxHeight)
--   print(im:size())
--   os.exit()
  im = image.scale(im,targetW, targetH, 'simple')
  im = image.rotate(im, rotRad)
  
  -- bookkeeping
  local transformInfo = {}
  transformInfo.scaleH = targetH/bxHeight
  transformInfo.scaleW = targetW/bxWidth
  transformInfo.rotRad = rotRad
  
  return im, transformInfo
end

---------------------------------------------------------------------------------
--- Takes cropped (grayscale) segmentation image and places it into full frame (1920x1080)
function unwarpImage(im, cropInfo)
  -- rotate back
  im = image.rotate(im, -cropInfo.transformInfo.rotRad)
  im = image.scale(im, cropInfo.cropBox[5], cropInfo.cropBox[6], 'simple') -- upscale (nearest neighb.)
  
  -- create canvas and fill cropped region
  local fullIm = torch.ones(1080,1920):byte() * params.num_classes -- fill with 'unknown' class
--   fullIm[{{cropInfo.cropBox[2],cropInfo.cropBox[4]-1},{cropInfo.cropBox[1],cropInfo.cropBox[3]-1}}] = im -- ??? 
  fullIm[{{cropInfo.cropBox[2],cropInfo.cropBox[4]},{cropInfo.cropBox[1],cropInfo.cropBox[3]}}] = im
  
  return fullIm
end

---------------------------------------------------------------------------------
--- Takes cropped tensor of N classes (1st dim) and places it into full frame (1920x1080)
function unwarpTensor(tensor, cropInfo)
  local N = tensor:size(1) -- number of channels  
  
  local fullTensor = torch.zeros(N, 1080,1920)-- fill with zero prob
  fullTensor:narrow(1,N,1):fill(1) -- fill 'unknown class' plane with 1

  
  -- rotate back   
  for dim = 1,N do	-- handle each channel as one grayscale image
    local layer = tensor[{{dim},{},{}}]:clone() 		-- one slice
    layer = image.rotate(layer, -cropInfo.transformInfo.rotRad) -- rotate
    layer = image.scale(layer, cropInfo.cropBox[5], cropInfo.cropBox[6], 'simple') -- upscale
    
    fullTensor[{{dim},{cropInfo.cropBox[2],cropInfo.cropBox[4]},{cropInfo.cropBox[1],cropInfo.cropBox[3]}}] = layer
  end
  
  return fullTensor
end

---------------------------------------------------------------------------------
--- Converts the network output (N x C) to a multi-slice tensor (C x H x W)
function vecToTensor(vec)
  local c = vec:size(2)	-- number of classes  
  local imW, imH = 1, vec:size(1)
  if vec:size(1) == 320*240 then imW, imH = 320, 240
  elseif vec:size(1) == 640*480 then imW, imH = 640, 480
  elseif vec:size(1) == 768*576 then imW, imH = 768, 576
  elseif vec:size(1) == 960*540 then imW, imH = 960, 540
  elseif vec:size(1) == 1440*1080 then imW, imH = 1440, 1080
  elseif vec:size(1) == 1920*1080 then imW, imH = 1920, 1080
  end

  -- NOTE: We need to transpose first to reshape correctly
  return vec:t():reshape(c, imH, imW)
end

---------------------------------------------------------------------------------
--- Take the depth channel and convert to 3-channel disparity
function prepDepth(depths)
  if depths:nDimension() == 2 then
    depths = reshape(1, depths:size(2), depths:size(3))
  end
--   depths = depths / (2^16-1);
  
  local disp=torch.ones(depths:size()):float():cdiv(depths:float());
  disp[depths:eq(0)]=0;
--   disp=disp:div(torch.max(disp));
--   disp[disp:gt(35)]=0
--   %         disp(disp>0.001)=0; 

--   disp=disp/max(disp(:));  
--   disp=disp:div(torch.max(disp));
  
  local disp3=torch.zeros(3,depths:size(2),depths:size(3)) -- for now
  for c=1,3 do disp3[{{c},{},{}}]=disp:clone() end
  
  return disp3
end


--------------------------------------------------------------------------
--- Set global parameters, either from command line, or passed from ROS
function setParams(cuda_device, crop_mode, model_name, num_classes, obj_class, setting, packagePath)
  params = {}
  
  -- defaults
  local dp, ds, jit, verbose, epochs, densecap = 'apc_data', 4, 0, 2, 45, 0
  
  if  runFromCmdLine() then -- if executed from command line
    cmd = torch.CmdLine()
    cmd:text('Training')
    cmd:text()
    cmd:option('-data_path','apc_data')
    cmd:option('-cuda_device',0)
    cmd:option('-cuda_device_train',1)
    cmd:option('-num_classes',41)
    cmd:option('-obj_class',0)
    cmd:option('-setting','shelf')
    cmd:option('-downsample',ds)
    cmd:option('-jitter',jit)
    cmd:option('-epochs',epochs)
    cmd:option('-verbose',2)
    cmd:option('-mode','test')
    cmd:option('-crop_mode',3) -- 1=640x480 on binmask, 2=640x480 centered, 3=960x540 full
    cmd:option('-model_name','apc_hires')
    cmd:option('-feature_type','hha')
    cmd:option('-data_file','apc_hires')
    cmd:option('-data_suffix','','Optional suffix')
    cmd:option('-densecap',0)
    cmd:option('-im_dir','')
    cmd:option('-suppress_x',0) -- ignore all visualizations
    cmd:option('-dropout',0.25) -- dropout rate
    cmd:option('-learning_rate',0.001) -- learning rate
    cmd:option('-train_split','train') -- which split to use for training train/all
    cmd:option('-momentum',0.9)
    cmd:option('-lr_decay',0.0001)
    cmd:option('-dc_setting',1,'1=82x41 params, 2=2x41 params, 3=no params')
    
    params = cmd:parse(arg or {})
    if params.epochs == 0 then params.epochs = nil end
    
    if params.im_dir == '' then
      params.im_dir = nil
    end
    params.packagePath = '../'
  else -- passed from ROS

    params.cuda_device = cuda_device
    params.crop_mode = crop_mode
    params.num_classes = num_classes
    params.obj_class = obj_class
    params.setting = setting
    params.packagePath = packagePath    
    
    -- Rest is hard coded
    params.data_path = 'XXX' -- REMOVE?
    params.downsample = ds
    params.jitter = jit
    params.verbose = verbose
    params.mode = 'test'
    params.model_name = model_name
    params.epochs = nil
    params.densecap = 0
    params.suppress_x = 1
    
  end
  
  if params.num_classes >= 41 then params.obj_class = 0 end -- segment all classes
  
end



function readImage(im_dir)
  if im_dir == nil then
    local datasetDir = getDatasetDir()
    local captureDir = '20160424_hard/bin_3_2'
    local instDir = 'image004'
    im_dir = datasetDir..'/'..params.setting..'/'..captureDir..'/'..instDir..'/'
  end

  
  local local_rgbIm, local_featureIm, local_boxMask = rgbIm, featureIm, boxMask
--   print(rgbIm)
  
  -- FIXME this global solution isnt pretty
  if rgbIm == nil then local_rgbIm = image.load(im_dir..'rgb.png') end  
  if featureIm == nil then
    if params.feature_type == 'depths' then
      local_featureIm = image.load(im_dir..'depth.png') 
    elseif params.feature_type == 'hha' then
      local_featureIm = image.load(im_dir..'feature_hha.png') 
    elseif params.feature_type == 'rha' then
      local_featureIm = image.load(im_dir..'feature_hha.png') 
      local_rmdfeatureIm = image.load(im_dir..'feature_relativeMeanDiff.png')    
      local_featureIm[{{1},{},{}}] = local_rmdfeatureIm
    end
  end
  if boxMask == nil then local_boxMask = image.load(im_dir..'mask_box.png') end
  
  -- densecap
  local dcprobs = nil
  if params.densecap ~= 0 or params.dc_setting == 3 then
    dcprobs = readDensecap()
  end
  
  return local_rgbIm, local_featureIm, local_boxMask, dcprobs
end

--------------------------------------------------------------------------
--- Process input. Determine crop, scale, rotate, normalize, etc...
-- @param rgbIm 	...
function processInput(rgbIm, featureIm, boxMask, dcprobs)
  
  
--   printImageStats(rgbIm)
--   printImageStats(featureIm)
  local imW, imH = boxMask:size(3), boxMask:size(2)
  local firstLeft, lastRight, firstTop, lastBottom = getBinmaskBBox(boxMask)
  local bxWidth, bxHeight = lastRight-firstLeft, lastBottom-firstTop  
  local aR = bxWidth/bxHeight
  
  local bxTop, bxLeft, bxBottom, bxRight = 1,1,1080,1920

   -- %%%%%    VARIANT I   %%%%%  
   -- put a 4:3 box on the ground plane of the mask_box window
  if params.crop_mode == 1 then
    if aR > 4/3 then -- keep height, cut off sides
	local bxCenterX = torch.round(firstLeft + bxWidth/2);
	bxWidth = torch.round(bxHeight * 4 / 3); -- new width
	bxLeft = torch.round(bxCenterX - bxWidth/2);
	bxRight = torch.round(bxCenterX + bxWidth/2);
    else -- keep width, clip top of box
	bxHeight = torch.round(bxWidth * 3 / 4);
	bxBottom = firstTop + bxHeight;
    end
  elseif params.crop_mode == 2 then
  -- %%%%%    VARIANT II   %%%%%
   -- crop from left/right, putting it in the h-center of the mask_box
    local maskCenter = torch.round(firstLeft + (lastRight - firstLeft)/2);
    bxTop, bxBottom = 1, imH;
    bxLeft = maskCenter-720+1; 
    bxRight = maskCenter+720;

    if bxLeft<1 then    bxRight=bxRight+torch.abs(bxLeft)+1; bxLeft = 1; end -- move right if necessary
    if bxRight>imW then bxLeft=bxLeft-(bxRight-imW); bxRight=imW; end -- move left if necessary
    
    bxWidth = bxRight - bxLeft+1; bxHeight = bxBottom-bxTop+1;
  else
    -- %%%%%    VARIANT III   %%%%%
    -- full frame
    bxTop, bxLeft, bxBottom, bxRight = 1,1,1080,1920
    bxWidth = bxRight - bxLeft+1; bxHeight = bxBottom-bxTop+1; 
  end  
  aR = bxWidth/bxHeight;
    

  print(string.format('Box: %d, %d, %d, %d. (%d x %d). Ratio = %.2f\n', 
    bxLeft, bxTop, bxRight, bxBottom, bxWidth, bxHeight, aR))

  -- save crop info log
  local cropInfo = {}
  cropInfo.datasetDir = datasetDir
  cropInfo.captureDir = captureDir
  cropInfo.instDir = instDir
  cropInfo.rgbIm = rgbIm
  cropInfo.cropBox = torch.Tensor({bxLeft, bxTop, bxRight, bxBottom, bxWidth, bxHeight, aR})

  
  rgbIm, cropInfo.transformInfo = prepImage(rgbIm, bxLeft, bxTop, bxWidth, bxHeight)
  featureIm = prepImage(featureIm, bxLeft, bxTop, bxWidth, bxHeight)
  local dc = nil
  if params.densecap ~= 0 or params.dc_setting == 3 then
    dc = prepImage(dcprobs, bxLeft, bxTop, bxWidth, bxHeight)
--     local ds = params.downsample
--     local ds = 1
--     local dcIH, dcIW = dc:size(2)/ds, dc:size(3)/ds
--
--     local newDC = torch.zeros(dc:size(1), dcIH, dcIW)
--     for c=1,newDC:size(1) do
--       newDC[{{c},{},{}}] = image.scale(dc[{{c},{},{}}]:squeeze(), dcIW, dcIH)
-- --       dci = showImage(newDC[{{c},{},{}}], dci)
-- --       sleep(1)
--
--     end
--     dc = newDC
  end
--   printImageStats(featureIm)
  
  -- if all we have is depth (1 channel), make it 3-channel disparity
  if featureIm:size(1) == 1 then featureIm = prepDepth(featureIm) end 
--   printImageStats(featureIm)
--   local hha3 = featureIm:clone()
  
  -- FIXME!
  local mean = torch.Tensor({118.81890055339, 96.374683896019, 69.991637137277})/(255) -- torch image.load returns [0,1] range
  local stdv = torch.Tensor({46.772151583782, 48.999432037339, 56.335641667327})/(255)
  
  local meanD = torch.Tensor({1.6701525449753, 1.6701525449753, 1.6701525449753})
  local stdvD = torch.Tensor({0.98780930042267, 0.98780930042267, 0.98780930042267})
  
--   local meanD = torch.Tensor({70.784103393555, 38.500095367432, 78.091682434082})/255
--   local stdvD = torch.Tensor({54.77409362793, 44.339385986328, 60.703144073486})/255
  
  -- NEW. TRY TO LOAD INFO
  infoFilename = getNormInfoFilename()
  print(infoFilename)
  if lfs.attributes(infoFilename) then
    pm('Loading normalization constants from '..infoFilename .. '...')
    local normInfo = torch.load(infoFilename)
    mean = normInfo.normDataInfo:narrow(2,1,1):squeeze()
    stdv = normInfo.normDataInfo:narrow(2,2,1):squeeze()
    meanD = normInfo.normFeaturesInfo:narrow(2,1,1):squeeze()
    stdvD = normInfo.normFeaturesInfo:narrow(2,2,1):squeeze()
--     print(mean, stdv, meanD, stdvD)
  end
  
  
  
  cropInfo.normInfo = mean:cat(stdv,2)
  cropInfo.normDInfo = meanD:cat(stdvD,2)
  
  
  for i=1,3 do -- over each image channel
    rgbIm[{{i},{},{}}]:add(-mean[i])
    rgbIm[{{i},{},{}}]:div(stdv[i])
    featureIm[{{i},{},{}}]:add(-meanD[i])
    featureIm[{{i},{},{}}]:div(stdvD[i])      
  end  
  
  printImageStats(rgbIm)
  printImageStats(featureIm)
  
--   print(test_data:size())
  -- throw to GPU if needed
  if params.cuda_device > 0 then 
    rgbIm = rgbIm:cuda()
    featureIm = featureIm:cuda()
    if params.densecap ~= 0 or params.dc_detting == 3 then 
      dc = dc:cuda() 
    end
  end  
  
  return rgbIm, featureIm, cropInfo, dc
end

--- Determine extents of the bin mask
-- @param mask 	binary image of the bin mask
function getBinmaskBBox(mask)
  local imW, imH = mask:size(3), mask:size(2)
  
  local verSum = mask:sum(2):squeeze() -- vertical sum (across rows), i.e. result length = im width
  local horSum = mask:sum(3):squeeze() -- vertical sum (across rows), i.e. result length = im width
  local firstLeft, lastRight, firstTop, lastBottom = 1, imW, 1, imH -- full frame

  -- for c=2,verSum:nElement() do
  local i=1
  while firstLeft==1 and i<=verSum:nElement() and verSum[1]==0 do  i=i+1
    if verSum[i-1]==0 and verSum[i]~=0 then firstLeft = i end  
  end
  local i=lastRight
  while lastRight==imW and i>1 and verSum[-1]==0 do i=i-1
    if verSum[i+1]==0 and verSum[i]~=0 then lastRight = i end  
  end
  local i=1
  while firstTop==1 and i<=horSum:nElement() and horSum[1]==0 do  i=i+1
    if horSum[i-1]==0 and horSum[i]~=0 then firstTop = i end  
  end
  local i=lastBottom
  while lastBottom==imH and i>1 and horSum[-1]==0 do i=i-1
    if horSum[i+1]==0 and horSum[i]~=0 then lastBottom = i end  
  end  
  return firstLeft, lastRight, firstTop, lastBottom
end



function setImgPath(path)
  params.im_dir = path
end


--------------------------------------------------------------------------
--- takes maximum along the classes dimension to convert raw prediction to class index
-- @param	output	Either NxC or CxHxW tensor
-- @param 	selectClasses	(optional) keep only these
function outputToMask(output, selectClasses)  
  if output:nDimension() == 2 then 
    output = vecToTensor(output) -- make tensor first if needed
  end
  
  -- error handling for selectClasses
  if selectClasses ~= nil then
    if torch.max(selectClasses) <= output:size(1) then
      output = output:index(1, selectClasses:long())
    else
      print('WARNING. Selected classes do not exist! Ignoring...')
    end
  end
  local _,labelMask = torch.max(output,1)
  local imW, imH = labelMask:size(3), labelMask:size(2)

  -- return a HxW mask
  return labelMask:squeeze():byte()
end

--------------------------------------------------------------------------
--- Get a list of present class IDs from directory mask_*.png annotation files
-- DEPRECATED
-- function getRelevantClasses(clNames)
--   clNames = clNames or classes_names
--   local trimClasses, trimClassesNames = {}, {}
--   for f in lfs.dir(params.im_dir) do
--     local _,className = string.match(f, '(mask)_([%a+%d+_]+)') -- search for mask_* pattern    
--     if className ~= nil and
-- 	className ~= 'segmentation' and 
-- 	className ~= 'combined' and 
-- 	className ~= 'container' and 
-- 	className ~= 'estimated' and
-- 	className ~= 'shelf' 
-- 	then
--       
-- --       pm(className)
--       table.insert(trimClasses, classNameToID(className, clNames))
-- --       table.insert(trimClassesNames, className)
--     end
--   end
--   table.insert(trimClasses, classNameToID('unknown', clNames))
-- --   table.insert(trimClassesNames, 'unknown')
--   -- abort()
--   trimClasses = torch.Tensor(trimClasses):long()
--   return trimClasses, trimClassesNames
-- end

--------------------------------------------------------------------------
--- Get a list of present class IDs from directory yaml annotation file
-- @param       clNames Existing class names
function getRelevantClasses(clNames)
  clNames = clNames or classes_names
  local trimClasses, trimClassesNames = {}, {}
  
  local filename = params.im_dir .. '/polygons.yaml'
  if fileExists(filename) then                          -- if annotation file found
    for line in io.lines(filename) do                   -- iterate over all lines
      local className = string.match(line, 'name:%s*([%a+%d+_]+)') -- find line of polygon name
      if className ~= nil then
        local classID = classNameToID(className, clNames)
        if not table.contains(trimClasses, classID) then -- ignore duplicates
          table.insert(trimClasses, classID)  
        end
      end
    end    
  else    
    print('WARNING! Annotation file '..filename..' not found. Considering all classes.')
    for k,v in pairs(clNames) do
      table.insert(trimClasses, classNameToID(v, clNames))
    end
  end
  table.insert(trimClasses, classNameToID('unknown', clNames))
  
  trimClasses = torch.Tensor(trimClasses):long()
--   print(trimClasses)
  return trimClasses, trimClassesNames
end

--------------------------------------------------------------------------
--- Returns whether a table contains an element
function table.contains(table, element)
  for _, value in pairs(table) do
    if value == element then return true end
  end
  return false
end


function setObjectList(clsNames)
--   for k,v in pairs(clsNames) do print(k,v) end
  
  local trimClasses = {}
  for k,v in pairs(clsNames) do
    table.insert(trimClasses, classNameToID(v, classes_names))    
  end
--   table.insert(trimClasses, classNameToID('unknown', classes_names))
--   for k,v in pairs(trimClasses) do print(k,v) end
  trimClasses = torch.Tensor(trimClasses):long()
  return trimClasses
end

--------------------------------------------------------------------------
--- Print image statistics for debugging purposes
function printImageStats(im)
  pm(string.format('Min: %.2f \t Max: %.2f \t Mean: %.2f \t Std: %.2f',
    im:min(), im:max(), im:mean(), im:std()), 2) -- 3 = verbosity debug
end

function evalSegmentation(gt, res)
  gt,res = gt:byte(), res:byte()
  local true_positives = torch.sum(torch.cmul(gt,res))
  local relevant = gt:sum()
  local positives = res:sum()
  
  local precision = true_positives / positives
  if positives == 0 then precision = 0 end
  local recall = true_positives / relevant
  local f1 = 0 
  if precision+recall ~= 0 then 
    f1 = 2 * (precision*recall) / (precision+recall)
  end
  
  
  return precision, recall, f1
end

-- function getClassesIndex(tab)
--   local ret = torch.IntTensor(#tab)
--   for k,v in pairs(tab) do
--     
--   end
-- end

--------------------------------------------------------------------------
--- Overlay legend for semantic classes. Assumes global classes_names 
-- @param img   original image
-- @param cls   table of class IDs
function drawCaptions(img, cls)
  local textX, textY, lineHeight = 20, 10, 20
  for k=1,cls:nElement() do
    local id = cls[k]
    local col = getColorFromID(k)
    
    -- overlay text
    img = image.drawText(img, classes_names[id], textX, textY+(k-1)*lineHeight,
      {color = {col[1], col[2], col[3]}, size = 2, inplace=false})
      
    -- fill rectangle
    for c=1,3 do
      img:narrow(1,c,1):narrow(2,textY+2+(k-1)*lineHeight,lineHeight-4):narrow(3,1,textX-5):fill(col[c])
    end
  end
  return img
end


function freadFloat(file)
  file:seekEnd()
  print(file:position())
  local n = (file:position() - 1) / 4 / 4
  if n == 0 then
    return nil
  end

  local tensor = torch.FloatTensor(n, 4)

  file:seek(1)
  assert(file:readFloat(tensor:storage()) == n*4)
  return tensor
end


--------------------------------------------------------------------------
--- Read DenseCap probabilities, return fullframe ClxHxW tensor
function readDensecap(clNames)
  clNames = clNames or classes_names
  local DENSECAP_ALPHA = 0.75
  local dcprobs = torch.Tensor(params.num_classes,1080,1920):fill(1.0 - DENSECAP_ALPHA)
  for f in lfs.dir(params.im_dir) do
    local _,className = string.match(f, '(prob)_([%a+%d+_]+)(.png)') -- search for prob_* pattern    
    if className ~= nil and 
	className ~= 'container' and 
	className ~= 'estimated' and
	className ~= 'shelf' 
	then
	  
      local id = classNameToID(className, clNames)
      local probfile = string.format('%s/%s',params.im_dir, f)
      local dc = image.load(probfile)
      dcprobs[{{id},{},{}}]:add(DENSECAP_ALPHA * dc)
    end
  end
  return dcprobs
end

--------------------------------------------------------------------------
--- Weighted sum of two images
function blendAdditive(im1, im2, alpha)
  
  local alphablend = alpha or 0.5 -- weight of first im
  local olIm = (im1 * alphablend) + (im2 * (1-alphablend))
  return olIm
end

--------------------------------------------------------------------------
--- determine edges in an image (neighboring pixel difference)
function findEdges(im)
  if im:nDimension() == 3 then im = im:mean(1):squeeze() end -- rgb to grey
  
  local imH, imW = im:size(1), im:size(2)
  local edges = torch.zeros(imH, imW)
  for y=2,imH-1 do
    for x=2,imW-1 do
      if im[y][x] ~= im[y-1][x] or im[y][x] ~= im[y][x-1] or im[y][x] ~= im[y-1][x-1] 
      or im[y][x] ~= im[y+1][x] or im[y][x] ~= im[y][x+1] or im[y][x] ~= im[y+1][x+1]  then
        edges[y][x] = 1
      end
    end
  end
  return edges
end
