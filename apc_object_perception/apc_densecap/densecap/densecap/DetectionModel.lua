require 'torch'
require 'nn'
require 'nngraph'

require 'densecap.LanguageModel'
require 'densecap.DenseCapModel'
require 'densecap.LocalizationLayer'
require 'densecap.modules.BoxRegressionCriterion'
require 'densecap.modules.BilinearRoiPooling'
require 'densecap.modules.ApplyBoxTransform'
require 'densecap.modules.LogisticCriterion'
require 'densecap.modules.PosSlicer'

local box_utils = require 'densecap.box_utils'
local utils = require 'densecap.utils'


local DetectionModel, parent = torch.class('DetectionModel', 'nn.Module')

function DetectionModel:__init(opt)
  opt.num_classes = utils.getopt(opt, 'num_classes', 39)
  opt.depth = utils.getopt(opt, 'depth', 0)
  opt.std = utils.getopt(opt, 'std', 0.01) -- Used to initialize new layers
  opt.use_cluster_anchors = utils.getopt(opt, 'cluster_anchors', false)

  opt.classifier = utils.getopt(opt, 'classifier', 'softmax') -- choices: svm, sigmoid, softmax, iou_reg

  -- Train on negative samples?
  opt.use_negative_samples = utils.getopt(opt, 'use_negative_samples', true)

  -- Use pretrained DenseCap model?
  opt.pretrained_model = utils.getopt(opt, 'pretrained_model', 'data/models/densecap/densecap-pretrained-vgg16.t7')

  self.opt = opt

  -- Load full DenseCap model
  if opt.pretrained_model ~= '' then
    densecap = torch.load(opt.pretrained_model).model
  else
    opt.vocab_size = 1000
    opt.seq_length = 100
    opt.idx_to_token = {}
    densecap = DenseCapModel(opt)
  end

  if cudnn then
    print(string.format("========================= USING CUDNN (backend %s) ========================", opt.backend))
    if opt.backend == 'cudnn' then
      cudnn.convert(densecap.net, cudnn)
      cudnn.convert(densecap.nets.localization_layer.nets.rpn, cudnn)
    else
      densecap.net:cuda()
      densecap.nets.localization_layer.nets.rpn:cuda()
    end
  end

  -- Setup our own structure

  -- This will hold the whole model
  self.net = nn.Sequential()
  self.nets = {}

  -- CNN
  self.nets.conv_net1 = densecap.nets.conv_net1
  self.nets.conv_net2 = densecap.nets.conv_net2

  if opt.depth ~= 0 then
    if opt.depth_cnn == 'concat' then
      print('Concat HHA')
    elseif opt.depth_cnn == 'rgb' then
      print('\nCopying RGB CNN to depth CNN')
      self.nets.depth_net = nn.Sequential()
      self.nets.depth_net:add(densecap.nets.conv_net1:clone())
      self.nets.depth_net:add(densecap.nets.conv_net2:clone())
    elseif opt.depth_cnn ~= '' then
      print('\nLoading separately trained depth CNN from', opt.depth_cnn)
      self.nets.depth_net = torch.load(opt.depth_cnn)
      if opt.backend == 'cudnn' then
        cudnn.convert(self.nets.depth_net, cudnn)
      else
        self.nets.depth_net:cuda()
      end
    end
  end

  -- Localization layer
  self.nets.localization_layer = densecap.nets.localization_layer

  -- Recognition networks parts
  self.nets.recog_base = densecap.nets.recog_base
  self.nets.objectness_branch = densecap.nets.objectness_branch
  self.nets.box_reg_branch = densecap.nets.box_reg_branch

  -- We may need to adapt the localization layer and the recog_base network
  -- to accept the additional depth feature maps
  if opt.depth ~= 0 then
    print('\nAdapting RPN to accept depth')

    local old_rpn_conv = self.nets.localization_layer.nets.rpn.modules[1]
    print(old_rpn_conv)
    local rpn_input_size = 2*old_rpn_conv.nInputPlane
    if opt.depth_cnn == 'concat' then
      rpn_input_size = old_rpn_conv.nInputPlane + 3
    end
    local new_rpn_conv = nn.SpatialConvolution(
      rpn_input_size,
      old_rpn_conv.nOutputPlane,
      old_rpn_conv.kW, old_rpn_conv.kH,
      old_rpn_conv.dW, old_rpn_conv.dH,
      old_rpn_conv.padW, old_rpn_conv.padH
    )

    -- Transfer weights, initialize new weights from normal distribution
    print("RPN weights:", new_rpn_conv.weight:size())
    print("old: ", old_rpn_conv.weight:size())

    new_rpn_conv.weight:fill(0.0)
    new_rpn_conv.weight:narrow(2, 1, old_rpn_conv.nInputPlane):copy(0.5*old_rpn_conv.weight)

    if opt.depth_cnn ~= 'concat' then
      new_rpn_conv.weight:narrow(2, old_rpn_conv.nInputPlane + 1, new_rpn_conv.weight:size(2) - old_rpn_conv.nInputPlane):copy(0.5*old_rpn_conv.weight) --:normal(0, opt.std)
    else
      new_rpn_conv.weight:narrow(2, old_rpn_conv.nInputPlane + 1, 3):normal(0, opt.std)
    end

    if cudnn and opt.backend == 'cudnn' then
      cudnn.convert(new_rpn_conv, cudnn)
    else
      new_rpn_conv:cuda()
    end

    self.nets.localization_layer.nets.rpn.modules[1] = new_rpn_conv

    -- Adapt the recog_base net
    print('\nAdapting recog_base to accept depth')
    print(self.nets.recog_base)

    local old_recog = self.nets.recog_base.modules[2]
    local ll = self.nets.localization_layer
    local recog_base_input = 2*ll.opt.input_dim
    if opt.depth_cnn == 'concat' then
      recog_base_input = ll.opt.input_dim + 3
    end

    local new_recog = nn.Linear(
      recog_base_input * ll.opt.output_width * ll.opt.output_height,
      4096
    )

    -- Transfer weights, initialize new weights from normal distribution
    new_recog.weight:narrow(2, 1, old_recog.weight:size(2)):copy(0.5*old_recog.weight)
    if opt.depth_cnn == 'concat' then
      new_recog.weight:narrow(2, old_recog.weight:size(2)+1, new_recog.weight:size(2) - old_recog.weight:size(2)):normal(0, opt.std)
    else
      new_recog.weight:narrow(2, old_recog.weight:size(2)+1, new_recog.weight:size(2) - old_recog.weight:size(2)):copy(0.5*old_recog.weight)
    end

    self.nets.recog_base.modules[2] = new_recog
  end

  local HIDDEN_DIM1 = 2048
  local HIDDEN_DIM2 = 1024
  self.nets.classifier1 = nn.Linear(4096 + 1, HIDDEN_DIM1)
--  self.nets.classifier1 = nn.Linear(4096 + 1, opt.num_classes)
  self.nets.classifier1.weight:normal(0, opt.std)
  self.nets.classifier1.bias:zero()

  self.nets.classifier2 = nn.Linear(HIDDEN_DIM1, HIDDEN_DIM2)
  self.nets.classifier2.weight:normal(0, opt.std)
  self.nets.classifier2.bias:zero()

  self.nets.classifier3 = nn.Linear(HIDDEN_DIM2, opt.num_classes)
  self.nets.classifier3.weight:normal(0, opt.std)
  self.nets.classifier3.bias:zero()

  self.nets.classifier = nn.Sequential()
  self.nets.classifier:add(self.nets.classifier1)
  self.nets.classifier:add(nn.LeakyReLU(0.1))
  self.nets.classifier:add(self.nets.classifier2)
  self.nets.classifier:add(nn.LeakyReLU(0.1))
  self.nets.classifier:add(self.nets.classifier3)


  if opt.classifier == 'sigmoid' or opt.classifier == 'iou_reg' then
    self.nets.softmax = nn.Sigmoid()
  elseif opt.classifier == 'softmax' then
    self.nets.softmax = nn.LogSoftMax()
  elseif opt.classifier == 'svm' then
    self.nets.softmax = nn.Identity()
  else
    error("Unknown classifier selected: " .. opt.classifier)
  end

  -- Recognition network
  -- We want to surgically remove the language generation part (RNN), but keep
  -- everything else. Unfortunately, this means that we have to rebuild the
  -- graph here.

  local roi_feats = nn.Identity()()
  local roi_boxes = nn.Identity()()
  local gt_boxes = nn.Identity()()
  local gt_labels = nn.Identity()()

  local roi_codes = self.nets.recog_base(roi_feats)
  local objectness_scores = self.nets.objectness_branch(roi_codes)

  local pos_roi_codes = nn.PosSlicer(){roi_codes, gt_labels}
  local pos_roi_boxes = nn.PosSlicer(){roi_boxes, gt_boxes}

  local final_box_trans = self.nets.box_reg_branch(pos_roi_codes)
  local final_boxes = nn.ApplyBoxTransform(){pos_roi_boxes, final_box_trans}

  local roi_codes_with_score = nn.JoinTable(1,1){roi_codes, objectness_scores}
  local classifier_output = self.nets.classifier(roi_codes_with_score)
  local softmax_output = self.nets.softmax(classifier_output)

  -- Annotate nodes
  roi_codes:annotate{name='recog_base'}
  objectness_scores:annotate{name='objectness_branch'}
  pos_roi_codes:annotate{name='code_slicer'}
  pos_roi_boxes:annotate{name='box_slicer'}
  final_box_trans:annotate{name='box_reg_branch'}
  classifier_output:annotate{name='classifier'}
  softmax_output:annotate{name='softmax'}

  local inputs = {roi_feats, roi_boxes, gt_boxes, gt_labels}
  local outputs = {
    objectness_scores,
    pos_roi_boxes, final_box_trans, final_boxes,
    gt_boxes, gt_labels,
    softmax_output
  }
  self.nets.recognition_network = nn.gModule(inputs, outputs)

  graph.dot(self.nets.recognition_network.fg, 'detection_recog', 'detection_recog')

  -- Final structure
  if opt.depth == 0 then
    print("\nNot using depth.")

    -- Boring CNN
    self.net:add(self.nets.conv_net1)
    self.net:add(self.nets.conv_net2)
  else
    print("\nUsing depth!")

    -- Our Input is now 1x6xHxW with 3 RGB channels and 3 HHA channels.
    local rgbhha = nn.Identity()()

    -- Split into RGB and HHA (both 1x3xHxW)
    local rgb = nn.Narrow(2, 1, 3)(rgbhha)
    local hha = nn.Narrow(2, 4, 3)(rgbhha)

    -- Calculate CNN features on rgb
    local cnn_feat = self.nets.conv_net2(self.nets.conv_net1(rgb))

    local combined = nil
    if opt.depth_cnn == 'concat' then
      -- Downsample HHA to the same feature map dimension
      local down_hha = nn.SpatialSubSampling(3, 1, 1, 16, 16)(hha)

      combined = nn.JoinTable(1,3)({cnn_feat, down_hha})
    else
      -- Calculate CNN features on depth
      local hha_feat = self.nets.depth_net(hha)

      -- Concat CNN features and HHA (=> 1x1024xhxw)
      combined = nn.JoinTable(1,3)({cnn_feat, hha_feat})
    end

    local module = nn.gModule({rgbhha}, {combined})
    self.net:add(module)
  end

  self.net:add(self.nets.localization_layer)
  self.net:add(self.nets.recognition_network)

  -- Set up Criterions
  self.crits = {}
  self.crits.objectness_crit = nn.LogisticCriterion()
  self.crits.box_reg_crit = nn.BoxRegressionCriterion(opt.end_box_reg_weight)

  if self.opt.classifier == 'sigmoid' then
    self.crits.sigmoid_crit = nn.BCECriterion()
  elseif self.opt.classifier == 'softmax' then
    self.crits.softmax_crit = nn.ClassNLLCriterion()
  elseif self.opt.classifier == 'svm' then
    self.crits.svm_crit = nn.SoftMarginCriterion()
  elseif self.opt.classifier == 'iou_reg' then
    self.crits.iou_reg_crit = nn.BCECriterion()
  end

  self:training()
  self.finetune_cnn = false
end

function DetectionModel:training()
  parent.training(self)
  self.net:training()
end

function DetectionModel:evaluate()
  parent.evaluate(self)
  self.net:evaluate()
end


--[[
Set test-time parameters for this DetectionModel.

Input: Table with the following keys:
- rpn_nms_thresh: NMS threshold for region proposals in the RPN; default is 0.7.
- final_nms_thresh: NMS threshold for final predictions; default is 0.3.
- num_proposals: Number of proposals to use; default is 1000
--]]
function DetectionModel:setTestArgs(kwargs)
  self.nets.localization_layer:setTestArgs{
    nms_thresh = utils.getopt(kwargs, 'rpn_nms_thresh', 0.7),
    max_proposals = utils.getopt(kwargs, 'num_proposals', 1000)
  }
  self.opt.final_nms_thresh = utils.getopt(kwargs, 'final_nms_thresh', 0.3)
end


--[[
Convert this DetectionModel to a particular datatype, and convert convolutions
between cudnn and nn.
--]]
function DetectionModel:convert(dtype, use_cudnn)
  self:type(dtype)
  if cudnn and use_cudnn ~= nil then
    local backend = nn
    if use_cudnn then
      backend = cudnn
    end
    cudnn.convert(self.net, backend)
    cudnn.convert(self.nets.localization_layer.nets.rpn, backend)
  end
end


--[[
Run the model forward.

Input:
- image: Pixel data for a single image of shape (1, 3, H, W)

After running the model forward, we will process N regions from the
input image. At training time we have access to the ground-truth regions
for that image, and assume that there are P ground-truth regions. We assume
that the language model has a vocabulary of V elements (including the END
token) and that all captions have been padded to a length of L.

Output: A table of
- objectness_scores: Array of shape (N, 1) giving (final) objectness scores
  for boxes.
- pos_roi_boxes: Array of shape (P, 4) at training time and (N, 4) at test-time
  giving the positions of RoI boxes in (xc, yc, w, h) format.
- final_box_trans: Array of shape (P, 4) at training time and (N, 4) at
  test-time giving the transformations applied on top of the region proposal
  boxes by the final box regression.
- final_boxes: Array of shape (P, 4) at training time and (N, 4) at test-time
  giving the coordinates of the final output boxes, in (xc, yc, w, h) format.
- gt_boxes: At training time, an array of shape (P, 4) giving ground-truth
  boxes corresponding to the sampled positives. At test time, an empty tensor.
- gt_labels: At training time, an array of shape (P, L) giving ground-truth
  sequences for sampled positives. At test-time, and empty tensor.
--]]
function DetectionModel:updateOutput(input)
  -- Make sure the input is (1, 3, H, W)
  assert(input:nDimension() == 4 and input:size(1) == 1)

  if self.opt.depth == nil or self.opt.depth == 0 then
    assert(input:size(2) == 3)
  else
    assert(input:size(2) == 6)
  end

  local H, W = input:size(3), input:size(4)
  self.nets.localization_layer:setImageSize(H, W)

  if self.train then
    assert(not self._called_forward,
      'Must call setGroundTruth before training-time forward pass')
    self._called_forward = true
  end
  self.output = self.net:forward(input)

  -- At test-time, apply NMS to final boxes
  local verbose = false
  if verbose then
    print(string.format('before final NMS there are %d boxes', self.output[4]:size(1)))
    print(string.format('Using NMS threshold of %f', self.opt.final_nms_thresh))
  end
  if not self.train and self.opt.final_nms_thresh > 0 then
    -- We need to apply the same NMS mask to the final boxes and their
    -- objectness scores, and the output from the language model

    local class_scores_float = self.output[1]:float()
    local final_boxes_float = self.output[4]:float()

    local boxes_scores = torch.FloatTensor(final_boxes_float:size(1), 5)
    local boxes_x1y1x2y2 = box_utils.xcycwh_to_x1y1x2y2(final_boxes_float)
    boxes_scores[{{}, {1, 4}}]:copy(boxes_x1y1x2y2)
    boxes_scores[{{}, 5}]:copy(class_scores_float[{{}, 1}])
--    boxes_scores[{{}, 5}]:copy(self.output[7]:float()[{{}, 4}])

    -- Get NMS mask
    local idx = box_utils.nms(boxes_scores, self.opt.final_nms_thresh)

    self.output[1] = class_scores_float:index(1, idx):typeAs(self.output[1])
    self.output[4] = final_boxes_float:index(1, idx):typeAs(self.output[4])
    self.output[7] = self.output[7]:index(1, idx)
  end

  return self.output
end


--[[
Run a test-time forward pass, plucking out only the relevant outputs.

Input: Tensor of shape (1, 3, H, W) giving pixels for an input image.

Returns:
- final_boxes: Tensor of shape (N, 4) giving coordinates of output boxes
  in (xc, yc, w, h) format.
- objectness_scores: Tensor of shape (N, 1) giving objectness scores of
  those boxes.
- classes: Tensor of shape (N, 1) (0: background)
--]]
function DetectionModel:forward_test(input, label_mask)
  self:evaluate()
  local output = self:forward(input)
  local final_boxes = output[4]
  local objectness_scores = output[1]

  -- If we have a label mask, multiply with it
  if label_mask then
    local scores = output[7]

    local PUNISHMENT = -1000.0

    local delta = PUNISHMENT * (1.0 - label_mask)
    delta = delta:typeAs(scores)
    delta = delta:view(1, delta:size(1))
    delta = delta:expandAs(scores)

    output[7] = torch.add(delta, scores)
  end

  local max_score, max_index = torch.max(output[7], 2)

  -- Filter out very weak classifications ( p < 0.1 )
  if false then
    local mask = objectness_scores:select(2,1):ge(-10.0)
    local indices = torch.linspace(1, final_boxes:size(1), final_boxes:size(1)):long()[mask:byte()]

    if indices:nDimension() == 0 then
      return torch.Tensor(), torch.Tensor(), torch.Tensor(), {}, torch.Tensor(), {}
    end

    print(indices:size())
    final_boxes = final_boxes:index(1, indices)
    objectness_scores = objectness_scores:index(1, indices)
    max_score = max_score:index(1, indices)
    max_index = max_index:index(1, indices)
  end

  local labels = {}
  for i = 1, final_boxes:size(1) do
--     labels[i] = self.opt.label_strings[max_index[i][1]]
    labels[i] = string.format("%s (%f, %f)", self.opt.label_strings[max_index[i][1]], max_score[i][1], objectness_scores[i][1])
--    labels[i] = string.format("%s (%f)", self.opt.label_strings[max_index[i][1]], max_score[i][1])
  end

  local class_confidences
  if self.opt.use_negative_samples == false then -- NOTE: use_negative_samples may be nil on older checkpoints
--     print("Multiplying with objectness score")
    class_confidences = torch.cmul(objectness_scores:view(final_boxes:size(1), 1):expand(final_boxes:size(1), self.opt.num_classes), output[7])
  else
--     print("Using raw class confidences")
    class_confidences = output[7]
  end

  return final_boxes, objectness_scores, max_index, labels, class_confidences
end


function DetectionModel:setGroundTruth(gt_boxes, gt_labels)
  self.gt_boxes = gt_boxes
  self.gt_labels = gt_labels
  self._called_forward = false
  self.nets.localization_layer:setGroundTruth(gt_boxes, gt_labels)
end


function DetectionModel:backward(input, gradOutput)
  -- Manually backprop through part of the network
  -- self.net has 4 elements:
  -- (1) CNN part 1        (2) CNN part 2
  -- (3) LocalizationLayer (4) Recognition network
  -- We always backprop through (3) and (4), and only backprop through
  -- (2) when finetuning; we never backprop through (1).
  -- Note that this means we break the module API in this method, and don't
  -- actually return gradients with respect to our input.

  local start_idx = 4
  local end_idx = 2

  -- FIXME: We should write this out explicitly instead of iterating, that
  -- actually would make it clearer.
  if self.opt.depth == nil or self.opt.depth == 0 then
    end_idx = 3
    start_idx = 4
    if self.finetune_cnn then end_idx = 2 end
  else
    end_idx = 2
    start_idx = 3
    if self.finetune_cnn then end_idx = 1 end
--     assert(not self.finetune_cnn, "Cannot finetune CNN in depth == 1 mode")
  end

  local dout = gradOutput
  for i = start_idx, end_idx, -1 do
    local layer_input
    if i == 1 then
      layer_input = input
    else
      layer_input = self.net:get(i-1).output
    end

    dout = self.net:get(i):backward(layer_input, dout)
  end

  self.gradInput = dout
  return self.gradInput
end

--[[
We naughtily override the module's getParameters method, and return:
- params: Flattened parameters for the RPN and recognition network
- grad_params: Flattened gradients for the RPN and recognition network
- cnn_params: Flattened portion of the CNN parameters that will be fine-tuned
- grad_cnn_params: Flattened gradients for the portion of the CNN that will
  be fine-tuned.
--]]
function DetectionModel:getParameters()
  local fakenet = nn.Sequential()
  fakenet:add(self.nets.localization_layer)
  fakenet:add(self.nets.recognition_network)
  return fakenet:getParameters()
end

function DetectionModel:getCNNParameters()
  if self.opt.depth == 1 then
    return self.net:get(1):getParameters()
  else
    return self.net:get(2):getParameters()
  end
end

function DetectionModel:getClassifierParameters()
  return self.nets.classifier:getParameters()
end

function DetectionModel:clearState()
  self.net:clearState()
  for k, v in pairs(self.crits) do
    if v.clearState then
    v:clearState()
    end
  end
end

--[[
Perform a (training-time) forward pass to compute output and loss,
and a backward pass to compute gradients.

This is a nonstandard method, but it allows the DetectionModel to
have control over its own Criterions.

Input: data is table with the following keys:
- image: 1 x 3 x H x W array of pixel data
- gt_boxes: 1 x B x 4 array of ground-truth object boxes
  TODO: What format are the boxes?
- gt_labels: 1 x B array of ground-truth labels for boxes
--]]
function DetectionModel:forward_backward(data)
  self:training()

  -- Run the model forward
  self:setGroundTruth(data.gt_boxes, data.gt_labels)
  local out = self:forward(data.image)

  -- Pick out the outputs we care about
  local objectness_scores = out[1]
  local pos_roi_boxes = out[2]
  local final_box_trans = out[3]
  local gt_boxes = out[5] -- WARNING: This changes if we re-add classifier output
  local gt_labels = out[6]
  local softmax_output = out[7]

  local num_boxes = objectness_scores:size(1)
  local num_pos = pos_roi_boxes:size(1)

  -- Compute final objectness loss and gradient
  local objectness_labels = torch.LongTensor(num_boxes):zero()
  objectness_labels[{{1, num_pos}}]:fill(1)
  local end_objectness_loss = self.crits.objectness_crit:forward(
    objectness_scores, objectness_labels
  )

  end_objectness_loss = end_objectness_loss * self.opt.end_objectness_weight
  local grad_objectness_scores = self.crits.objectness_crit:backward(
    objectness_scores, objectness_labels
  )
  grad_objectness_scores:mul(self.opt.end_objectness_weight)

  -- Compute box regression loss; this one multiplies by the weight inside
  -- the criterion so we don't do it manually.
  local end_box_reg_loss = self.crits.box_reg_crit:forward(
    {pos_roi_boxes, final_box_trans},
    gt_boxes
  )
  local din = self.crits.box_reg_crit:backward(
    {pos_roi_boxes, final_box_trans},
    gt_boxes
  )
  local grad_pos_roi_boxes, grad_final_box_trans = unpack(din)

  -- Compute classification loss
  local grad_softmax_output
  local softmax_loss

  if self.opt.use_negative_samples then -- train on negative examples
    local class_labels = torch.LongTensor(num_boxes):fill(1) -- background code
    class_labels[{{1, num_pos}}]:copy(gt_labels)

    if self.opt.classifier == 'sigmoid' then
      indices = class_labels:view(-1,1):typeAs(softmax_output)
      one_hot = torch.zeros(class_labels:size(1), self.opt.num_classes):typeAs(softmax_output)
      one_hot:scatter(2, indices, 1)

      softmax_loss = self.opt.captioning_weight * self.crits.sigmoid_crit:forward(softmax_output, one_hot)
      grad_softmax_output = self.crits.sigmoid_crit:backward(softmax_output, one_hot)
    elseif self.opt.classifier == 'softmax' or self.opt.classifier == nil then
      class_labels = class_labels:typeAs(softmax_output)

      softmax_loss = self.opt.captioning_weight * self.crits.softmax_crit:forward(softmax_output, class_labels)
      grad_softmax_output = self.crits.softmax_crit:backward(softmax_output, class_labels)
    elseif self.opt.classifier == 'svm' then
      indices = class_labels:view(-1,1):typeAs(softmax_output)
      svm_targets = torch.Tensor(class_labels:size(1), self.opt.num_classes):fill(-1.0):typeAs(softmax_output)
      svm_targets:scatter(2, indices, 1)

      softmax_loss = self.opt.captioning_weight * self.crits.svm_crit:forward(softmax_output, svm_targets)
      grad_softmax_output = self.crits.svm_crit:backward(softmax_output, svm_targets)
    elseif self.opt.classifier == 'iou_reg' then

      -- Compute IoU between predictions and gt boxes

      local pred_area = torch.cmul(pos_roi_boxes[{{}, 3}], pos_roi_boxes[{{}, 4}])
      local gt_area = torch.cmul(gt_boxes[{{}, 3}], gt_boxes[{{}, 4}])

      local pred = box_utils.xcycwh_to_x1y1x2y2(pos_roi_boxes)
      local gt = box_utils.xcycwh_to_x1y1x2y2(gt_boxes)

      local x0 = torch.cmax(pred:select(2,1), gt:select(2,1))
      local y0 = torch.cmax(pred:select(2,2), gt:select(2,2))
      local x1 = torch.cmin(pred:select(2,3), gt:select(2,3))
      local y1 = torch.cmin(pred:select(2,4), gt:select(2,4))

      local w = (x1 - x0):cmax(0)
      local h = (y1 - y0):cmax(0)

      local intersection = torch.cmul(w,h)

      local ious = torch.cdiv(intersection, pred_area + gt_area - intersection)

      -- Predict zero for all negative rects...
      local target = torch.zeros(class_labels:size(1), self.opt.num_classes):typeAs(softmax_output)

      -- ... and the IoU for positive rects!
      local indices = gt_labels:view(-1,1):typeAs(softmax_output)
      target[{{1,num_pos}}]:scatter(2, indices, ious:view(-1,1))

      softmax_loss = self.opt.captioning_weight * self.crits.iou_reg_crit:forward(softmax_output, target)
      grad_softmax_output = self.crits.iou_reg_crit:backward(softmax_output, target)
    end

    grad_softmax_output:mul(self.opt.captioning_weight)
  else -- do not train on negative examples
    local pos_softmax_output = softmax_output[{{1, num_pos}}]

    local class_labels = gt_labels:select(2,1):typeAs(softmax_output)
    if class_labels:size(1) > 0 then
      if self.opt.classifier == 'sigmoid' then
        indices = gt_labels:view(-1,1):typeAs(softmax_output)
        one_hot = torch.zeros(class_labels:size(1), self.opt.num_classes):typeAs(softmax_output)
        one_hot:scatter(2, indices, 1)

        softmax_loss = self.opt.captioning_weight * self.crits.sigmoid_crit:forward(pos_softmax_output, one_hot)
        grad_softmax_output = self.crits.sigmoid_crit:backward(pos_softmax_output, one_hot)
      elseif self.opt.classifier == 'softmax' or self.opt.classifier == nil then
        softmax_loss = self.crits.softmax_crit:forward(pos_softmax_output, class_labels)
        softmax_loss = softmax_loss * self.opt.captioning_weight
        grad_softmax_output = self.crits.softmax_crit:backward(pos_softmax_output, class_labels)
        grad_softmax_output:mul(self.opt.captioning_weight)
      else
        error("implement me")
      end
    else
      print("warn: no softmax update")
      softmax_loss = 0.0
      grad_softmax_output = torch.Tensor(num_pos, self.opt.num_classes):fill(0.0):typeAs(softmax_output)
    end
    print()
    print(softmax_loss)

    local tmp_grad_softmax_output = torch.Tensor(num_boxes, self.opt.num_classes):fill(0.0):typeAs(softmax_output)
    tmp_grad_softmax_output[{{1,num_pos}}] = grad_softmax_output
    grad_softmax_output = tmp_grad_softmax_output
  end

  local ll_losses = self.nets.localization_layer.stats.losses
  local losses = {
    mid_objectness_loss=ll_losses.obj_loss_pos + ll_losses.obj_loss_neg,
    mid_box_reg_loss=ll_losses.box_reg_loss,
    end_objectness_loss=end_objectness_loss,
    end_box_reg_loss=end_box_reg_loss,
    softmax_loss=softmax_loss
  }
  local total_loss = 0
  for k, v in pairs(losses) do
    total_loss = total_loss + v
  end
  losses.total_loss = total_loss

  -- Run the model backward
  local grad_out = {}
  grad_out[1] = grad_objectness_scores
  grad_out[2] = grad_pos_roi_boxes
  grad_out[3] = grad_final_box_trans
  grad_out[4] = out[4].new(#out[4]):zero()
  grad_out[5] = gt_boxes.new(#gt_boxes):zero()
  grad_out[6] = gt_labels.new(#gt_labels):zero()
  grad_out[7] = grad_softmax_output

  self:backward(input, grad_out)

  return losses
end

function DetectionModel:setAnchors(anchors)
  local makeAnchors = self.nets.localization_layer.nets.rpn:findModules('nn.MakeAnchors')[1]
  print('old anchors:')
  print(makeAnchors.anchors)
  print('new anchors:')
  makeAnchors.anchors = anchors:typeAs(makeAnchors.anchors)
  print(makeAnchors.anchors)
end
