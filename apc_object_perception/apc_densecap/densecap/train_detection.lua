--[[
Main entry point for training a DenseCap model
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
print(opt)
torch.setdefaulttensortype('torch.FloatTensor')
torch.manualSeed(opt.seed)
if opt.gpu >= 0 then
  -- cuda related includes and settings
  require 'cutorch'
  require 'cunn'

  if opt.backend == 'cudnn' then
    require 'cudnn'
  end
  cutorch.manualSeed(opt.seed)
  cutorch.setDevice(opt.gpu + 1) -- note +1 because lua is 1-indexed
end

-- initialize the data loader class
local loader = DataLoader(opt)
opt.label_strings = loader.info.label_strings
opt.num_classes = #opt.label_strings

print('number of classes: ', opt.num_classes)

print('Current memory usage:')
os.execute('nvidia-smi')

-- initialize the DenseCap model object
local dtype = 'torch.CudaTensor'
local model = models_detection.setup(opt):type(dtype)

-- If wanted, override anchors in RPN with cluster results
if opt.cluster_anchors ~= 0 then
  model:setAnchors(loader.cluster_anchors)
end

-- we start with the classifier
local params, grad_params = model:getParameters()

print('total number of parameters in model (excluding CNN): ', grad_params:nElement())
print('Current memory usage:')
os.execute('nvidia-smi')

-------------------------------------------------------------------------------
-- Loss function
-------------------------------------------------------------------------------
local loss_history = {}
local all_losses = {}
local results_history = {}
local test_results_history = {}
local train_results_history = {}
local learning_rate_history = {}
local iter = 0
local transformed = false
local img_info = nil

local function lossFun()
  model:training()

  -- Fetch data using the loader
  local timer = torch.Timer()
  local info
  local data = {}
  local loaderOpt = {transform=opt.augment, iterate=true}
  data.image, data.gt_boxes, data.gt_labels, info, data.region_proposals = loader:getBatch(loaderOpt)
  transformed = info[1].transformed
  for k, v in pairs(data) do
    data[k] = v:type(dtype)
  end
  if opt.timing then cutorch.synchronize() end
  local getBatch_time = timer:time().real

  -- Run the model forward and backward
  model.timing = opt.timing
  model.cnn_backward = false
  if opt.finetune_cnn_after ~= -1 and iter > opt.finetune_cnn_after then
    model.finetune_cnn = true
  end
  model.dump_vars = false
  if opt.progress_dump_every > 0 and iter % opt.progress_dump_every == 0 then
    model.dump_vars = true
  end
  local losses, stats = model:forward_backward(data)

  --+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  -- Visualization/Logging code
  --+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if opt.losses_log_every > 0 and iter % opt.losses_log_every == 0 then
    local losses_copy = {}
    for k, v in pairs(losses) do losses_copy[k] = v end
    loss_history[iter] = losses_copy
  end

  img_info = info[1]
  return losses, stats
end

-------------------------------------------------------------------------------
-- Main loop
-------------------------------------------------------------------------------
local loss0
local optim_state = {}
local cnn_optim_state = {}
local best_val_score = -1
local best_val_loss = nil

-- SGDR: restart after this many epochs
local Te = 200.0

while true do

  -- Reset gradients.
  -- NOTE: The gradient may also be modified by eval_utils.evaluate_split()
  -- below, so be careful when experimenting with these lines. No information
  -- from the validation split should enter the training itself.
  grad_params:zero()

  -- Compute loss and gradient
  local losses, stats = lossFun()

  -- Parameter update
  local learning_rate = opt.learning_rate
  -- SGDR
  if false then
    local itersPerEpoch = #loader.train_ix
    local itersPerRestart = itersPerEpoch * Te
    local restartNumber = math.floor(iter / itersPerRestart)
    local positionInRestartPeriod = (iter % itersPerRestart) / itersPerRestart

    -- dt: sin arg until restart
    local dt = math.pi
    local t0 = math.pi/2.0

    learning_rate = opt.learning_rate / (1 + restartNumber) * (1.0 + math.sin(t0 + positionInRestartPeriod * dt)) / 2.0
  end

  -- old fixed lr schedule
  if true then
    learning_rate = opt.learning_rate
    if iter >= 3000 and iter < 20000 then
      learning_rate = opt.learning_rate + (opt.learning_rate / 10 - opt.learning_rate) * (iter - 3000.0) / (20000.0 - 3000.0)
    elseif iter >= 20000 and iter < 40000 then
      learning_rate = opt.learning_rate / 10
    elseif iter >= 40000 and iter < 50000 then
      learning_rate = opt.learning_rate / 50
    elseif iter >= 50000 and iter < 60000 then
      learning_rate = opt.learning_rate / 100
    elseif iter >= 60000 and iter < 70000 then
      learning_rate = opt.learning_rate / 500
    end
  end

  if opt.losses_log_every > 0 and iter % opt.losses_log_every == 0 then
    learning_rate_history[iter] = learning_rate
  end

  -- Apply L2 regularization
  if opt.weight_decay > 0 then
    grad_params:add(opt.weight_decay * learning_rate, params)
  end

  if true then
    adam(params, grad_params, learning_rate, opt.optim_beta1,
      opt.optim_beta2, opt.optim_epsilon, optim_state)
  else
    sgdmom(params, grad_params, learning_rate, 0.9, optim_state)
  end

  -- print loss and timing/benchmarks
  transform_str = '  '
  if transformed == 1 then transform_str = 'H ' end
  if transformed == 2 then transform_str = ' V' end
  if transformdd == 3 then transform_str = 'HV' end
  io.write(string.format('\riter %d [%s] lr %f: %s [%s]                    ', iter, transform_str, learning_rate, utils.build_loss_string(losses), img_info.filename))
  io.flush()
  if opt.timing then print(utils.build_timing_string(stats.times)) end

  if ((opt.eval_first_iteration == 1 or iter > 0) and iter % opt.save_checkpoint_every == 0) or (iter+1 == opt.max_iters) then
    print('')
    model:evaluate()

    -- Set test-time options for the model
    model.nets.localization_layer:setTestArgs{
      nms_thresh=opt.test_rpn_nms_thresh,
      max_proposals=opt.test_num_proposals,
    }
    model.opt.final_nms_thresh = opt.test_final_nms_thresh

    -- Evaluate validation performance
    if opt.evaluate_val ~= 0 then
      local eval_kwargs = {
        model=model,
        loader=loader,
        split='val',
        max_images=opt.val_images_use,
        dtype=dtype,
      }
      local results = eval_utils.eval_split(eval_kwargs)
      -- local results = eval_split(1, opt.val_images_use) -- 1 = validation
      results_history[iter] = results
    end

    -- Evaluate test performance (for checking if the validation set is representative, I have my doubts)
    if opt.evaluate_test ~= 0 then
      eval_kwargs = {
        model=model,
        loader=loader,
        split='test',
        max_images=opt.val_images_use,
        dtype=dtype,
      }
      local test_results = eval_utils.eval_split(eval_kwargs)
      -- local results = eval_split(1, opt.val_images_use) -- 1 = validation
      test_results_history[iter] = test_results
    end

    -- Evaluate train performance (for verifying training progress)
    if opt.evaluate_train ~= 0 then
      eval_kwargs = {
        model=model,
        loader=loader,
        split='train',
        max_images=opt.val_images_use,
        dtype=dtype,
      }
      local train_results = eval_utils.eval_split(eval_kwargs)
      train_results_history[iter] = train_results
    end

    -- serialize a json file that has all info except the model
    local checkpoint = {}
    checkpoint.opt = opt
    checkpoint.iter = iter
    checkpoint.loss_history = loss_history
    checkpoint.results_history = results_history
    checkpoint.test_results_history = test_results_history
    checkpoint.train_results_history = train_results_history
    checkpoint.learning_rate_history = learning_rate_history

    cjson.encode_number_precision(5) -- number of sig digits to use in encoding
    cjson.encode_sparse_array(true, 2, 10)
    local text = cjson.encode(checkpoint)
    local file = io.open(opt.checkpoint_path .. '.json', 'w')
    file:write(text)
    file:close()
    print('wrote ' .. opt.checkpoint_path .. '.json')

    -- Only save t7 checkpoint if there is an improvement in mAP
    -- NO: Our validation sets are very small and not representative.
    -- In constrast, we never seem to overfit to the training set.
    -- So just use the last model.
    do
      checkpoint.model = model

      -- We want all checkpoints to be CPU compatible, so cast to float and
      -- get rid of cuDNN convolutions before saving
      model:clearState()
      model:float()
      if cudnn then
        cudnn.convert(model.net, nn)
        cudnn.convert(model.nets.localization_layer.nets.rpn, nn)
      end
      torch.save(opt.checkpoint_path, checkpoint)
      print('wrote ' .. opt.checkpoint_path)
      params = nil
      grad_params = nil
      collectgarbage()

      -- Now go back to CUDA and cuDNN
      if opt.gpu ~= -1 then
        model:cuda()
        if cudnn and opt.backend == 'cudnn' then
          cudnn.convert(model.net, cudnn)
          cudnn.convert(model.nets.localization_layer.nets.rpn, cudnn)
        end
      end

      -- All of that nonsense causes the parameter vectors to be reallocated, so
      -- we need to reallocate the params and grad_params vectors.
      collectgarbage()
      print('Restoring parameters...')
      params, grad_params = model:getParameters()
      print('Done. GC...')
      collectgarbage()
      print('Done.')
    end

    model:training()
  end

  -- stopping criterions
  iter = iter + 1
  -- Collect garbage every so often
  if true then collectgarbage() end
  if loss0 == nil then loss0 = losses.total_loss end
--[[  if losses.total_loss > loss0 * 100 then
    print('loss seems to be exploding, quitting.')
    break
  end--]]
  if opt.max_iters > 0 and iter >= opt.max_iters then break end
end

