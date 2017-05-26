-- kate: space-indent on; indent-width 2; mixedindent off; indent-mode cstyle;

require 'torch'		-- duh
require 'nn'		-- neural networks
require 'nngraph'
require 'optim'		-- optimization
require 'image'		-- image processing, load, save, scale, ...
require 'cunn'		-- cuda. required for GPU train/test
require 'lfs'		-- file system: create dir, etc.
require 'util.misc'	-- Anton's helper functions
require 'apc'		-- For test code

-- optnet = require 'optnet' -- optimize net

matio = require 'matio'  -- Matlab IO interface

--[[
Train image segmentation based on 
[Husain et al., Combining Semantic and Geometric Features 
for Object Class Segmentation of Indoor Scenes, ICRA 2016]
--]]

-- nngraph.setDebug(true)  -- uncomment for debug mode
torch.setdefaulttensortype('torch.FloatTensor')

function create_network(nb_output, upsample)
  local filter_size = 11 -- fixed filter size
  local pad = (filter_size-1) / 2
  local net = nn.Sequential();  -- make a multi-layer structure
  net:add(nn.SpatialConvolutionMM(256+96,128,filter_size,filter_size,1,1,pad)) -- (256+96) -> 96
  net:add(nn.PReLU())
  net:add(nn.Dropout(params.dropout))
  net:add(nn.SpatialConvolutionMM(128,64,filter_size,filter_size,1,1,pad)) -- 96 -> 32
  net:add(nn.PReLU())
  net:add(nn.SpatialConvolutionMM(64,nb_output,filter_size,filter_size,1,1,pad)) -- 32x120x160 -> Clx120x160
  net:add(nn.SpatialUpSamplingNearest(upsample))
  net:add(nn.Transpose({1,2},{2,3})) -- HxWxCl
  net:add(nn.Reshape(params.imHeight*params.imWidth, nb_output)) -- (H*W)xCl
  net:add(nn.LogSoftMax())
  if params.cuda_device > 0 then net:cuda() end
  return net
end


--------------------------------------------------------------------------
--- create a combination of semantic segmentation and densecap
-- Not working yet :(
function create_multimodal_network(nb_output,upsample)

  local filter_size = 11 -- fixed filter size
  local pad = (filter_size-1) / 2
  
  local filter2_size = 1 -- fixed filter2 size
  local pad2 = (filter2_size-1) / 2  

  local dsIW, dsIH = params.imWidth/upsample, params.imHeight/upsample
  
  local inputs = {}
  table.insert(inputs, nn.Identity()())
  table.insert(inputs, nn.Identity()())

  local inpIm = inputs[1] -- pretrained
  local dc = inputs[2]	-- densecap

  
  -- standard forward pass to before upsampling
  local n1 = nn.SpatialConvolutionMM(256+96,128,filter_size,filter_size,1,1,pad)(inpIm):annotate{name='convD1'} -- (256+96) -> 96
  n1 = nn.PReLU()(n1)
  n1 = nn.Dropout(params.dropout)(n1)
  n1 = nn.SpatialConvolutionMM(128,64,filter_size,filter_size,1,1,pad)(n1):annotate{name='convD2'} -- 96 -> 32
  n1 = nn.PReLU()(n1)
  n1 = nn.SpatialConvolutionMM(64,nb_output,filter_size,filter_size,1,1,pad)(n1):annotate{name='convD3'} -- 32x120x160 -> Clx120x160

  local intermediate = nn.SpatialUpSamplingNearest(upsample)(n1)
  intermediate = nn.Transpose({1,2},{2,3})(intermediate)
  intermediate = nn.Reshape(params.imHeight*params.imWidth, nb_output, false)(intermediate)
  intermediate = nn.LogSoftMax()(intermediate)

--   concat densecap and do one convolution
  local n2 = nil
  -------
  if params.dc_setting == 1 then
  
    local cat = nn.JoinTable(1){n1, nn.CMulTable(){n1, dc}}
    n2 = nn.SpatialConvolutionMM(2*nb_output, nb_output, filter2_size, filter2_size, 1, 1, pad2)(cat):annotate{name='convE1'}

  elseif params.dc_setting == 2 then
    local allMM = {}
    for c = 1,params.num_classes do
      local n1c = nn.Narrow(1,c,1)(n1)
      local dcc = nn.Narrow(1,c,1)(dc)
      local spl = nn.JoinTable(1){n1c, nn.CMulTable(){n1c,dcc}}
      local convc = nn.SpatialConvolutionMM(2,1,filter2_size, filter2_size, 1, 1, pad2)(spl):annotate{name=string.format('convE%d',c)}
      table.insert(allMM, convc)
    end
    n2 = nn.JoinTable(1)(allMM)
    
    -------
  end
  n2 = nn.SpatialUpSamplingNearest(upsample)(n2)
  n2 = nn.Transpose({1,2},{2,3})(n2)
  n2 = nn.Reshape(params.imHeight*params.imWidth, nb_output)(n2)
  n2 = nn.LogSoftMax()(n2)
  
  local outputs = {}
  table.insert(outputs, n2)
  table.insert(outputs, intermediate)
  

  gmod = nn.gModule(inputs, outputs)
  if params.cuda_device > 0 then gmod:cuda() end
  return gmod

end

 
-- Train a Neural Network
function train_network()

  local parameters,gradParameters = network:getParameters()
  network:training()	-- set flag for dropout
  
  local bs = 1 -- batchsize
  -- hyper-parameters
  local lR = params.learning_rate / torch.sqrt(bs) 
  local optimConfig = {learningRate = params.learning_rate,
    momentum = params.momentum,
    learningRateDecay = params.lr_decay}
    
  -- count number of train instances
  local nfiles = train_data.imArray:size(1)

  print( "Training the network with "..nfiles.." files and "..parameters:nElement().. " parameters" )
  local weights = torch.Tensor(#classes):fill(1)

  -- set loss weights anti-proportional to frequency
  -- TODO: Try median frequency ratio!
  for c=1,#classes-1 do  weights[c] = 1-train_ratios[c] end    
  if params.setting == 'shelf' then weights[#classes] = 0 end -- unknown class (only ignore for shelf because objects outside are not annotated. For box, there are no objects outside box)
  
--   weights[#classes] = 0
--   weights[2] = 0
--   local criterion = nn.ClassNLLCriterion(weights)
  local critNLL =  nn.ClassNLLCriterion(weights)
  criterion = critNLL
  if params.densecap~=0 then
    criterion = nn.ParallelCriterion()    
    criterion:add(critNLL)
    criterion:add(critNLL)
  end
  
  if params.cuda_device >0 then criterion = criterion:cuda() end
  
  -- assign input and target  
  local inputs = train_data_clone.imArray
  local features = hha_train_clone.hha
  local targets = train_data.labelArray  
  
  

  for ep=1,params.epochs do
    -- do one epoch
    EPOCH = ep -- global TODO REMOVE
    shuffle = torch.randperm(nfiles)

    local timer = torch.Timer()

    for ii = 1,nfiles,bs do
      xlua.progress(ii, nfiles)
      timer:reset()

      local t = shuffle[ii]
      local color_image = inputs[{t,{1,3},{},{}}]
      local depth_image2 = features[{t,{},{},{}}]

      local target_image = targets[{t,{},{}}]

      local setupTime = timer:time().real
      timer:reset()

      cutorch.setDevice(params.cuda_device_train)
      local input = network0:forward(color_image)                -- process RGB
      local inputd2 = networkd:forward(depth_image2):squeeze()   -- HHA 
      local input_concat = torch.cat(input,inputd2,1):squeeze()  -- concat RGB,  HHA
      cutorch.setDevice(params.cuda_device)
      collectgarbage('collect')
      input_concat = input_concat:clone():cuda()

      target = target_image:reshape(params.imWidth*params.imHeight) -- reshape target as vector

      local preprocTime = timer:time().real
      timer:reset()

      local DISPLAY = false

      -- create closure to evaluate f(X) and df/dX
      local loss = 0
      local losss = torch.zeros(2)
      local feval = function(x)

        -- get new parameters
        if x ~= parameters then parameters:copy(x) end
        collectgarbage()

        -- reset gradients
        gradParameters:zero()
        -- f is the average of all criterions
        -- evaluate function for complete mini batch

        local output = network:forward(input_concat) -- run forward pass
        local err = criterion:forward(output, target) -- compute loss
        loss = loss + err

        -- estimate df/dW
        local df_do = criterion:backward(output, target)

        network:backward(input_concat, df_do) -- update parameters

        if DISPLAY then
          -- display result
          win_input = showImage(color_image, win_input)
          win_target = showImage(colorizeLabels(target_image:float()), win_target)

          local _,predicted_labels = torch.max(output,2)
          predicted_labels = torch.reshape(predicted_labels:squeeze():float(),params.imHeight,params.imWidth)
          predicted_labels = colorizeLabels(predicted_labels:float()):float()
          win_pl = showImage(predicted_labels, win_pl)
        end

        local woohooString = ''
        if losss[1]<losss[2] then woohooString = 'Woohoo!' end
        pm(string.format('Training loss: %.4f, (%.4f\t%.4f) %s',loss, losss[1], losss[2], woohooString), 2)
  
        -- return loss and gradients
        return err,gradParameters
      end -- feval

      _,current_loss = optim.sgd(feval, parameters, optimConfig)
      collectgarbage('collect')

      local trainTime = timer:time().real
      timer:reset()
      print(string.format('setup: %fs, preproc: %fs, train: %fs',
        setupTime, preprocTime, trainTime
      ))
    end -- ii

    print(string.format('Loss: %.4f  Epoch: %d   grad-norm: %.4f',
      current_loss[1], ep, torch.norm(parameters)/torch.norm(gradParameters)))

    if (ep % 10) == 0 or ep == params.epochs then

    local filename = getModelFilename()
    if ep < params.epochs then filename = getModelFilename(ep) end

    pm('Saving checkpoint to '..filename .. '...')
    network:clearState()
    torch.save(filename, network)

    pm('Test...')
    network:training()
    end -- if epoch output
    collectgarbage('collect')
    
  end --epochs
  print('Training completed')
end

function printCombWeights(module)

  local wt = module.weight
  local bs = module.bias
  print(string.format('%8s%8s%8s%8s %s','Wt Seg','Wt DC','Wt ARst','Bias','Class'))
  for c=1,params.num_classes do
    local wtSeg = wt[c][c]
    local wtDC = wt[c][c+params.num_classes]
    local wtRest = wt[c]:abs():sum() - torch.abs(wtSeg) - torch.abs(wtDC)
    local bias = bs[c]
    print(string.format('%8.4f%8.4f%8.4f%8.4f %s',wtSeg,wtDC,wtRest,bias,classes_names[c]))
  end

end


function test_predictor_all(classes)
  print('Predict all images')
--   if params.mode=='train' then
--     testConfusion  = optim.ConfusionMatrix(classes)
--     testConfusion2 = torch.zeros(#classes,#classes)
--   end
  
  local bs=1 -- batch size
  network:evaluate()
  local nfiles = test_data.imArray:size(1)
--   local inputs = test_data.imArray
--   local features = hha_test.hha
  local inputs = test_data_clone.imArray
  local features = hha_test_clone.hha
  
  local targets = test_data.labelArray
  local accuracy = 0
  local predictions = torch.FloatTensor(nfiles,#classes,params.imHeight,params.imWidth)
  
  
  for t = 1,nfiles,bs do
    xlua.progress(t, nfiles)	-- progress bar
  
    
    local inpIm = inputs[{t,{1,3},{},{}}]
    local featIm = features[{t,{},{},{}}]
    local target = targets[{t,{},{}}]
    
    -- test one image
    cutorch.setDevice(params.cuda_device_train)
    local input = network0:forward(inpIm)	-- process RGB       
    local inputd2 = networkd:forward(featIm):squeeze()   -- HHA 
    local input_concat = torch.cat(input,inputd2,1):squeeze()  -- concat RGB,  HHA
    cutorch.setDevice(params.cuda_device)
    input_concat = input_concat:clone():cuda()
    
    if params.densecap ~= 0 then
      local dc = test_data.dcprobs[{{t}, {}, {}, {}}]:squeeze()
      input_concat = {input_concat, dc:cuda()}
    end
      
    local output = network:forward(input_concat)	-- run forward pass        
    
    local _,predicted_labels = torch.max(output,2)
    predicted_labels = torch.reshape(predicted_labels:squeeze():float(),params.imHeight,params.imWidth)
    -- per class masks
--     for o=1,params.num_classes do
--       local classIm = output:narrow(2,o,1):squeeze():float():reshape(params.imHeight,params.imWidth)
--       wi_classpl = image.display{image=classIm, win=wi_classpl, zoom=1}
--     end
    
    --    pl[{t,{},{}}] = predicted_labels:float()

    
--     if params.mode == 'train' then update_confusion2(testConfusion2, predicted_labels, target, classes) end
    
    if t==1 then
      target = colorizeLabels(target)
      predicted_labels = colorizeLabels(predicted_labels)
    end
    
    
--     win_input = showImage(inputs[{t,{1,3},{},{}}],win_input)
--     win_target = showImage(target,win_target)
--     win_pl = showImage(predicted_labels,win_pl)
--     if params.mode=='train' then
--       local imFileName = 'predictions/'..getModelSignature()..'-rgb.png'
--       image.save(imFileName, inputs[{t,{1,3},{},{}}])
--       local imFileName = 'predictions/'..getModelSignature()..'-pl.png'
--       image.save(imFileName, predicted_labels)    
--     end
    
    
    if params.mode == 'test' then
      local _,predicted_labels = torch.max(output,2)
      predicted_labels = torch.reshape(predicted_labels:squeeze():float(),params.imHeight,params.imWidth)      
      local unwarpedLabels = unwarpImage(predicted_labels, cropInfo.transformInfo, cropInfo)
      unwarpedLabels = colorizeLabels(unwarpedLabels)
--       win_pl_full = image.display{image = unwarpedLabels, win=win_pl_full, zoom=1}
--       showImage(unwarpedLabels, win_pl_full)

--       local imFileName = 'predictions/'..getModelSignature()..'-rgb.png'
      
       image.save('predictions/colored.png', unwarpedLabels)
       image.save('predictions/rgb.png', cropInfo.rgbIm)
    end
  end

--   if params.mode == 'train' then
--     testConfusion.mat = testConfusion2
--     if #classes < 15 then print(testConfusion) end
--     --  print('Test average accuracy = '..accuracy/nfiles)
--     print('Average Per-Class Accuracy [%] (test set) up4 = '..testConfusion.averageValid * 100)
--     print('Pixelwise Accuracy [%] (test set) up4 ='..testConfusion.totalValid * 100)
--     --  image.display(testConfusion:render()) -- render matrix
--     return testConfusion.averageValid * 100, testConfusion.totalValid * 100, predictions    
--   end
  print('done')
  
end

function update_confusion(testConfusion, predicted_labels, target, classes)
  -- loop over all locations
  local gt = torch.Tensor(#classes):zero()
  local output = torch.Tensor(#classes):zero()
  for y = 1,(#target)[1] do
    for x = 1,(#target)[2] do
      -- ground-truth at x,y location
      gt:zero()
      gt[target[{ y,x }]] = 1
      output:zero()
      output[predicted_labels[{ y,x }]] = 1
      -- update confusion matrix / error
      if target[{ y,x }] ~= #classes then
        testConfusion:add(output, gt)
      end
    end
  end
end

function update_confusion2(testConfusion2, predicted_labels, target, classes)
  -- loop over all locations
  confusion = torch.zeros(#classes,#classes)
  if params.cuda_device > 0 then 
    predicted_labels = predicted_labels:float() 
    target = target:float()
  end
  for i = 1,#classes-1 do
    for j = 1,#classes-1 do
      -- no. of predicted labels for class j
      np = torch.sum(torch.eq(target,i):cmul(torch.eq(predicted_labels,j)))
      confusion[{i,j}] = np
    end
  end
  testConfusion2:add(confusion)
end



function main()
  
  if runFromCmdLine() then setParams() end
  
    
  createAuxDirs() -- create required directories quietly
  
  
  pm('Running segmentation of '..params.num_classes..' classes on '..params.setting, 2)
  
  classes, classes_names = getClassInfo()
--   params.imHeight, params.imWidth = 480,640
--   network = create_multimodal_network(#classes, params.downsample)
--   print(network.modules)


--   if params.cuda_device>0 then cutorch.setDevice(params.cuda_device) end
  local dtype, use_cudnn = setup_gpus(params.cuda_device, 0)
  
  if params.mode=='train' then
    train_data, test_data, hha_train, hha_test = dofile('apc_dataset.lua')  
  else
    test_data, hha_test, cropInfo = processInput(params.im_dir)
    image.save('predictions/rgb.png', cropInfo.rgbIm)
    
    if params.cuda_device > 0 then 	
      test_data.imArray = test_data.imArray:cuda()
      test_data.labelArray = test_data.labelArray:cuda()
      hha_test.hha = hha_test.hha:cuda()
    end    
    
  end

  
  cutorch.setDevice(params.cuda_device_train)
  loadPretrainedModels() -- load OverFeat model parameters
  cutorch.setDevice(params.cuda_device)
    
  
  if params.mode == 'train' then  -- train mode. Create network and off we go...
    if params.densecap ~= 0 then
      network = create_multimodal_network(#classes, params.downsample)
      -- preset weights
      for k,v in ipairs(network.forwardnodes) do 
        mname = v.data.annotations.name
        
        if params.dc_setting == 1 then -- 41x41 params
          if mname == 'convE1' then
            v.data.module.weight:fill(0)
            v.data.module.bias:fill(0)
            print(v.data.module.weight:size())
            for c = 1,params.num_classes do
              v.data.module.weight[c][c] = .5
              v.data.module.weight[c][c+params.num_classes] = .5
            end
          end            
        elseif params.dc_setting == 2 then -- 2x41 params
          if mname~=nil and mname:len()>=5 and mname:sub(1,5) == 'convE' then
            v.data.module.weight:fill(.5)
            v.data.module.bias:fill(0)
          end                    
        end
      end

      
      graph.dot(network.fg, 'ForwardNet', 'MM_Network')

  
    else
      network = create_network(#classes, params.downsample)            
    end
    print("Created network with "..network:getParameters():nElement().. " parameters" )
    
    train_network()
  else	-- testing
  
    EPOCH = params.epochs -- this is necessary for model file name
    local networkFileName = getModelFilename()
    pm('Loading '..networkFileName)
    network = torch.load(networkFileName)	-- load the network
    if params.cuda_device == 0 then 
      network = network:float()
      print('converted to cpu')
    end
    
    collectgarbage()
    printGPUStatus('after garbage collection')
    
    test_predictor_all(network, test_data, hha_test, classes, params.downsample, params.jitter)
  end  
  
  -- keep display and terminate
  if runFromCmdLine() then sleep(1); os.exit()  end
end

-- if run on command line do main()
main()
