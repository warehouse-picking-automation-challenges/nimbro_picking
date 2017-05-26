-- kate: space-indent on; indent-width 2; mixedindent off; indent-mode cstyle;

local cjson = require 'cjson'
local utils = require 'densecap.utils'
local box_utils = require 'densecap.box_utils'
require 'image'
require 'paths'

local eval_utils = {}

-- Print contents of `tbl`, with indentation.
-- `indent` sets the initial level of indentation.
function tprint (tbl, indent)
  if not indent then indent = 0 end
  for k, v in pairs(tbl) do
    formatting = string.rep("  ", indent) .. k .. ": "
    if type(v) == "table" then
      print(formatting)
      tprint(v, indent+1)
    elseif type(v) == 'boolean' then
      print(formatting .. tostring(v))
    else
      print(formatting .. v)
    end
  end
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

--[[
Evaluate a DenseCapModel on a split of data from a DataLoader.

Input: An object with the following keys:
- model: A DenseCapModel object to evaluate; required.
- loader: A DataLoader object; required.
- split: Either 'val' or 'test'; default is 'val'
- max_images: Integer giving the number of images to use, or -1 to use the
  entire split. Default is -1.
- id: ID for cross-validation; default is ''.
- dtype: torch datatype to which data should be cast before passing to the
  model. Default is 'torch.FloatTensor'.
--]]
function eval_utils.eval_split(kwargs)
  local model = utils.getopt(kwargs, 'model')
  local loader = utils.getopt(kwargs, 'loader')
  local split = utils.getopt(kwargs, 'split', 'val')
  local max_images = utils.getopt(kwargs, 'max_images', -1)
  local id = utils.getopt(kwargs, 'id', '')
  local dtype = utils.getopt(kwargs, 'dtype', 'torch.FloatTensor')
  assert(split == 'val' or split == 'test' or split == 'train', 'split must be "val" or "test"')
  local split_to_int = {val=1, test=2,train=0}
  local proposal_dir = utils.getopt(kwargs, 'proposal_dir', '')
  local add_proposals = utils.getopt(kwargs, 'add_proposals', 0)
  local informed = utils.getopt(kwargs, 'informed', 0)
  split = split_to_int[split]
  print('using split ', split)

  if informed == 1 then
    print('Performing INFORMED evaluation')
  else
    print('Performing UNINFORMED evaluation')
  end

  model:evaluate()
  loader:resetIterator(split)
  local evaluator = DenseCaptioningEvaluator{id=id}

  local counter = 0
  local all_losses = {}

  local COMPUTE_LOSSES = false -- WARNING: If you switch this on, see below
  -- This flag causes model:forward_backward() to be called, which accumulates
  -- gradients. During training, this may have the effect that information
  -- from the validation set is actually used for training. You need to prevent
  -- that in train_detection.lua.

  while true do
    counter = counter + 1

    -- Grab a batch of data and convert it to the right dtype
    local data = {}
    local loader_kwargs = {split=split, iterate=true}
    local img, gt_boxes, gt_labels, info, _ = loader:getBatch(loader_kwargs)
    local data = {
      image = img:type(dtype),
      gt_boxes = gt_boxes:type(dtype),
      gt_labels = gt_labels:type(dtype),
    }
    info = info[1] -- Since we are only using a single image

    local frame_ok = true
    if proposal_dir and proposal_dir ~= '' then
      local frame = string.match(info.filename, '.*/shelf/(.*)/rgb.png')
      local proposal_filename = proposal_dir .. '/' .. frame .. '/single_saliency/boxes/frame_0.png_bboxes.txt'
      local proposal_file = io.open(proposal_filename, 'r')

      if not proposal_file then
        frame_ok = false
      else
        local boxes = {}
        for line in proposal_file:lines() do
          x,y,w,h = string.match(line, '(%d+) (%d+) (%d+) (%d+)')
          table.insert(boxes, {x,y,w,h})
        end

        local boxTensor = torch.FloatTensor(boxes)
        boxTensor = box_utils.xywh_to_xcycwh(boxTensor)
        -- upscale according to German
        boxTensor:mul(2.0)

        -- subtract orig bbox
        boxTensor[{{},1}]:csub(info.ori_bbox[1] - info.ori_bbox[3]/2)
        boxTensor[{{},2}]:csub(info.ori_bbox[2] - info.ori_bbox[4]/2)

        -- these are made on the rotated image
        boxTensor[{{},1}] = info.ori_bbox[3] - 1.0 - boxTensor[{{},1}]
        boxTensor[{{},2}] = info.ori_bbox[4] - 1.0 - boxTensor[{{},2}]

        -- now scale to image size
        local scale = info.width / info.ori_bbox[3]
        boxTensor:mul(scale)

        boxTensor, valid = box_utils.clip_boxes(boxTensor, {x_min=2.0, x_max=img[1]:size(3)-2, y_min=2.0, y_max=img[1]:size(2)-2}, 'xcycwh')

        indices = torch.linspace(1,boxTensor:size(1),boxTensor:size(1)):long()[valid]
        boxTensor = boxTensor:index(1, indices)

        -- aaaand override!
        model.nets.localization_layer.overrideBoxes = true
        model.nets.localization_layer.overrideBoxesData = boxTensor:cuda()

        -- debug
        local vis = img[1]:clone()
        -- swap RGB
        local perm = torch.LongTensor{3,2,1}
        vis = vis:index(1, perm)
        local boxes_draw = box_utils.xcycwh_to_x1y1x2y2(boxTensor)
        for k=1,boxes_draw:size(1) do
          image.drawRect(vis, boxes_draw[k][1], boxes_draw[k][2], boxes_draw[k][3], boxes_draw[k][4], {color={0,255,0}, lineWidth=2, inplace=true})
        end
        image.save(proposal_dir .. '/' .. frame .. '/boxes.png', vis)
      end
    end

    if add_proposals and add_proposals ~= 0 then
      print('Adding proposals from simple proposal generator')
      local directory = path.dirname(info.filename)
      local proposal_filename = path.join(directory, 'rgb.png.prop.bbox.bin')
      local file = torch.DiskFile(proposal_filename, "r", true)
      if file then
        file:binary()
        local bboxes = freadFloat(file)

        if bboxes then
          -- The data was originally intended for full crops of the rotated image.
          bboxes:mul(1920.0 / 720.0)
          bboxes[{{},1}] = 1920 - 1 - bboxes[{{},1}]
          bboxes[{{},2}] = 1080 - 1 - bboxes[{{},2}]

          -- subtract orig bbox
          bboxes[{{},1}]:csub(info.ori_bbox[1] - info.ori_bbox[3]/2)
          bboxes[{{},2}]:csub(info.ori_bbox[2] - info.ori_bbox[4]/2)

          -- these are made on the rotated image
          bboxes[{{},1}] = info.ori_bbox[3] - 1.0 - bboxes[{{},1}]
          bboxes[{{},2}] = info.ori_bbox[4] - 1.0 - bboxes[{{},2}]

          -- now scale to image size
          local scale = info.width / info.ori_bbox[3]
          bboxes:mul(scale)

          bboxes, valid = box_utils.clip_boxes(bboxes, {x_min=2.0, x_max=img[1]:size(3)-2, y_min=2.0, y_max=img[1]:size(2)-2}, 'xcycwh')

          indices = torch.linspace(1,bboxes:size(1),bboxes:size(1)):long()[valid]
          bboxes = bboxes:index(1, indices)

          -- aaaand override!
          model.nets.localization_layer.overrideBoxes = true
          model.nets.localization_layer.overrideBoxesData = bboxes:cuda()

          -- debug
          local vis = img[1]:clone()
          -- swap RGB
          local perm = torch.LongTensor{3,2,1}
          vis = vis:index(1, perm)
          local boxes_draw = box_utils.xcycwh_to_x1y1x2y2(bboxes)
          for k=1,boxes_draw:size(1) do
            image.drawRect(vis, boxes_draw[k][1], boxes_draw[k][2], boxes_draw[k][3], boxes_draw[k][4], {color={0,255,0}, lineWidth=2, inplace=true})
          end
          image.save(directory .. '/densecap_proposal_boxes.png', vis)
        else
          print('Warning: Could not read proposal file in directory', directory)
        end
      else
        print('Warning: Could not find proposal file in directory', directory)
      end
    end

    if frame_ok then

      if COMPUTE_LOSSES then
        -- Call forward_backward to compute losses
        model.timing = false
        model.dump_vars = false
        model.cnn_backward = false
        local losses = model:forward_backward(data)
        table.insert(all_losses, losses)
      end

      -- Call forward_test to make predictions, and pass them to evaluator
      local label_mask = torch.IntTensor(#loader.info.label_strings):fill(0)

      if informed == 1 then -- use information which classes are present?
        label_mask[1] = 1
        for i=1,gt_labels:size(2) do
          label_mask[gt_labels[1][i][1]] = 1
        end
      else
        label_mask = nil -- no prior.
      end

  --     timer = torch.Timer()
      local boxes, logprobs, classes, labels, class_confidences = model:forward_test(data.image, label_mask)
  --     print('Forward run took ' .. timer:time().real .. ' seconds')

      -- Do not remove background detections, for they may be actual positives
      -- with low confidence.
      if false then
        -- Remove background class
        if classes:nDimension() ~= 0 then
          mask = classes:int():ne(1):select(2,1)
          indices = torch.linspace(1, classes:size(1), classes:size(1)):long()[mask]
          if indices:nDimension() ~= 0 then
            boxes = boxes:index(1, indices)
            logprobs = logprobs:index(1, indices)
            classes = classes:index(1, indices)
      --      boxes = boxes[{{1,indices[1]}}]
      --      logprobs = logprobs[{{1,indices[1]}}]
      --      classes = classes[{{1,indices[1]}}]
          end
        end
      end

      if classes:nDimension() ~= 0 then
        evaluator:addResult(class_confidences, boxes, classes, gt_boxes[1], gt_labels[1])
      else
        print("No prediction left :-(")
        evaluator:addEmptyResult(gt_boxes[1], gt_labels[1])
      end

      -- Dump probability mask
      if false then
        local probMasks = {}
        local probMasksNormalizer = {}

--        local imsize = torch.LongStorage({info.ori_height, info.ori_width})
	print('WARNING: Forcing Full-HD (hack for wonky dataset)')
        local imsize = torch.LongStorage({1080,1920})
        local ori_bbox = info.ori_bbox

        -- for every bbox
        for i=1,classes:size(1) do

          local mask = torch.Tensor(imsize)
          local sigma_alpha = 0.5

          local x = nil
          local y = nil
          local w = nil
          local h = nil

          if ori_bbox ~= nil then
            x = ori_bbox[1] + ori_bbox[3]/2 - (boxes[i][1] * ori_bbox[3] / data.image:size(4))
            y = ori_bbox[2] + ori_bbox[4]/2 - (boxes[i][2] * ori_bbox[4] / data.image:size(3))
            w = boxes[i][3] * ori_bbox[3] / data.image:size(4)
            h = boxes[i][4] * ori_bbox[4] / data.image:size(3)
          else
            x = boxes[i][1] * info.ori_width / data.image:size(4)
            y = boxes[i][2] * info.ori_height / data.image:size(3)
            w = boxes[i][3] * info.ori_width / data.image:size(4)
            h = boxes[i][4] * info.ori_height / data.image:size(3)
          end

          print(x, y, w, h)

          image.gaussian({
            tensor=mask,
            mean_horz=x / imsize[2],
            mean_vert=y / imsize[1],
            sigma_horz=sigma_alpha * w / imsize[2],
            sigma_vert=sigma_alpha * h / imsize[1],
            amplitude=1.0
          })

          -- for every class
          for cls=2,#loader.info.label_strings do
      --      print(loader:getLabelStrings()[cls])

            if probMasks[cls] == nil then
              probMasks[cls] = torch.Tensor(imsize):fill(0.0)
              probMasksNormalizer[cls] = 0.0
            end

            local amplitude = math.exp(class_confidences[i][cls])
            probMasks[cls]:add(amplitude * mask)

            probMasksNormalizer[cls] = probMasksNormalizer[cls] + amplitude
          end
        end

        for class, mask in pairs(probMasks) do
          local name = loader:getLabelStrings()[class]
          local normalizer = torch.max(mask)
          local normalized = mask / normalizer
          image.save(string.format("%s/prob_%s.png", paths.dirname(info.filename), name), normalized)
        end
      end

      -- Print a message to the console
      local msg = '\rProcessed image %50s (%4d / %4d) of split %d, detected %4d regions   '
      local num_images = info.split_bounds[2]
      if max_images > 0 then num_images = math.min(num_images, max_images) end
      local num_boxes
      if classes:nDimension() ~= 0 then
        num_boxes = boxes:size(1)
      else
        num_boxes = 0
      end
      io.write(string.format(msg, info.filename, counter, num_images, split, num_boxes))
      io.flush()
      collectgarbage()
    end

    -- Break out if we have processed enough images
    if max_images > 0 and counter >= max_images then break end
    if info.split_bounds[1] == info.split_bounds[2] then break end
  end

  print('')

  local loss_results = {}
  if COMPUTE_LOSSES then
    loss_results = utils.dict_average(all_losses)
    print('Loss stats:')
    for k,v in pairs(loss_results) do
      print(string.format(' - %20s: %f', k, v))
    end
    print(loss_results)
    print('Average loss: ', loss_results.total_loss)
  end

  local ap_results, f1_results = evaluator:evaluate()
  print('AP breakdown:')
  tprint(ap_results)
  print(string.format('mAP: %f', 100 * ap_results.map))

  local out = {
    loss_results=loss_results,
    ap_results=ap_results,
    f1_results=f1_results
  }
  return out
end


local DenseCaptioningEvaluator = torch.class('DenseCaptioningEvaluator')
function DenseCaptioningEvaluator:__init(opt)
  self.id = utils.getopt(opt, 'id', '')

  self.classes = {}
end

function DenseCaptioningEvaluator:initClass(label)
  self.classes[label] = {
    records = {},
    confidences = {},
    ground_truth_positives = 0,
    bbox_precision = 0.0,
    bbox_recall = 0.0,
    bbox_count = 0
  }
end

function DenseCaptioningEvaluator:addEmptyResult(target_boxes, target_classes)
  for i=1,target_boxes:size(1) do
    local cls = target_classes[i][1]

    if self.classes[cls] == nil then
      self:initClass(cls)
    end

    -- Record the ground truth box
    self.classes[cls].ground_truth_positives = self.classes[cls].ground_truth_positives + 1
  end
end

-- boxes is (B x 4) are xcycwh, logprobs are (B x 2), target_boxes are (M x 4) also as xcycwh.
-- these can be both on CPU or on GPU (they will be shipped to CPU if not already so)
-- predict_text is length B list of strings, target_text is length M list of strings.
function DenseCaptioningEvaluator:addResult(class_confidences, boxes, classes, target_boxes, target_classes)
  assert(class_confidences:size(1) == boxes:size(1))
  assert(boxes:nDimension() == 2)

  self:addEmptyResult(target_boxes, target_classes)

  -- convert both boxes to x1y1x2y2 coordinate systems
  boxes = box_utils.xcycwh_to_x1y1x2y2(boxes)
  target_boxes = box_utils.xcycwh_to_x1y1x2y2(target_boxes)

  -- make sure we're on CPU
  boxes = boxes:float()
  class_confidences = class_confidences:float()
  target_boxes = target_boxes:float()

  for cls=2,class_confidences:size(2) do
    if self.classes[cls] == nil then
      self:initClass(cls)
    end
    self.classes[cls].seen = false
  end

  for i=1,target_classes:size(1) do
    -- Count one bounding box for each gt class
    -- Rationale: Someone retrieving objects is just interested in getting one object.
    local cls = target_classes[i][1]
    if not self.classes[cls].seen then
      self.classes[cls].bbox_count = self.classes[cls].bbox_count + 1
      self.classes[cls].seen = true
    end
  end

  -- Compute VOC mAP (region retrieval) and mean F1 score (pixel-based precision/recall for each image)
  -- for every non-background class:
  for cls=2,class_confidences:size(2) do

    -- 1. Sort detections by decreasing confidence
    local Y,IX = torch.sort(class_confidences:select(2,cls), 1, true) -- true makes order descending

    local nd = Y:size(1) -- number of detections
    local nt = target_boxes:size(1) -- number of gt boxes
    local used = torch.zeros(nt)

    local record = torch.IntTensor(nd)

    local f1_scored = false

    for d=1,nd do -- for each detection in descending order of confidence
      local ii = IX[d]
      local bb = boxes[ii]

      -- assign the box to its best match in true boxes
      local ovmax = 0.5 -- minimal IoU as per VOC evaluation
      local precision = 0
      local recall = 0
      local jmax = -1
      for j=1,nt do
        local bbgt = target_boxes[j]
        local bi = {math.max(bb[1],bbgt[1]), math.max(bb[2],bbgt[2]),
                    math.min(bb[3],bbgt[3]), math.min(bb[4],bbgt[4])}
        local iw = bi[3]-bi[1]+1
        local ih = bi[4]-bi[2]+1
        if target_classes[j][1] == cls and iw>0 and ih>0 then
          -- compute overlap as area of intersection / area of union
          local ua = (bb[3]-bb[1]+1)*(bb[4]-bb[2]+1)+
                  (bbgt[3]-bbgt[1]+1)*(bbgt[4]-bbgt[2]+1)-iw*ih
          local ov = iw*ih/ua
          if ov > ovmax then
            ovmax = ov
            jmax = j
            precision = (iw*ih) / ((bb[3]-bb[1]+1)*(bb[4]-bb[2]+1))
            recall = (iw*ih) / ((bbgt[3]-bbgt[1]+1)*(bbgt[4]-bbgt[2]+1))
          end
        end
      end

      local ok = 1
      if jmax ~= -1 and used[jmax] == 0 then
        used[jmax] = 1 -- mark as taken
        ok = 1
      else
        ok = 0
      end

      -- remember whether this detection is a true positive
      record[d] = ok

      -- if not done so yet, mark precision/recall for the best bbox
      if not f1_scored then
        self.classes[cls].bbox_precision = self.classes[cls].bbox_precision + precision
        self.classes[cls].bbox_recall = self.classes[cls].bbox_recall + recall
        f1_scored = true
      end
    end

    table.insert(self.classes[cls].records, record)
    table.insert(self.classes[cls].confidences, Y:double())
  end
end

function DenseCaptioningEvaluator:evaluate(verbose)
  if verbose == nil then verbose = true end

  local ap_results = {}

  -- The AP evaluation is separate for each class.
  for cls,class_data in pairs(self.classes) do
    -- Concat the data
    local confidences = torch.cat(class_data.confidences, 1)
    local records = torch.cat(class_data.records, 1)

    -- Sort by confidence over all images
    local confidences,ix = torch.sort(confidences, 1, true) -- descending

    -- build tp,fp arrays
    local tp = records:index(1, ix)
    local fp = 1 - tp

    print(string.format("True positives: %d, false positives: %d, gt positives: %d",
      tp:sum(), fp:sum(), class_data.ground_truth_positives
    ))

    -- calculate precision at every possible cut-off point
    tp = torch.cumsum(tp, 1):float()
    fp = torch.cumsum(fp, 1):float()
    local recall = torch.div(tp, class_data.ground_truth_positives)
    local precision = torch.cdiv(tp, tp + fp)

    local outfile = assert(io.open(string.format("/tmp/class_%d_pr.csv", cls), "w"))
    for i=1,recall:size(1) do
      outfile:write(string.format("%f %f\n", recall[i], precision[i]))
    end

    -- compute max-interpolated average precision
    local ap = 0
    local apn = 0
    for t=0,1,0.01 do
      local mask = torch.ge(recall, t):double()
      local precision_masked = torch.cmul(precision:double(), mask)
      local p = torch.max(precision_masked)
      ap = ap + p
      apn = apn + 1
    end
    ap = ap / apn

    ap_results[cls] = ap
  end

  local map = utils.average_values(ap_results)

  -- Bounding box F1 score per class
  print('Custom object localization F1-score:')
  local mean_f = 0.0
  local num_classes = 0
  local f1_breakdown = {}
  for cls,class_data in pairs(self.classes) do
    local n = class_data.bbox_count
    local precision = class_data.bbox_precision / n
    local recall = class_data.bbox_recall / n

    if n ~= 0 then
      local f1
      if precision + recall == 0.0 then
        f1 = 0.0
      else
        f1 = 2.0 * (precision * recall) / (precision + recall)
      end
      print(string.format('%20d (%3d samples): precision %3.3f, recall %3.3f, F-Score: %3.3f',
        cls, n, precision, recall, f1
      ))
      mean_f = mean_f + f1
      num_classes = num_classes + 1

      f1_breakdown[cls] = f1
    end
  end
  mean_f = mean_f / num_classes
  print('Mean F score: ', mean_f)

  -- lets get out of here
  local ap_results = {map = map, ap_breakdown = ap_results}
  local f1_results = {mean_f1 = mean_f, f1_breakdown = f1_breakdown}
  return ap_results, f1_results
end

return eval_utils
