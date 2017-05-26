require 'torch'
require 'nn'
require 'image'

require 'densecap.DetectionModel'
require 'densecap.DataLoader'
local utils = require 'densecap.utils'
local box_utils = require 'densecap.box_utils'
local vis_utils = require 'densecap.vis_utils'


--[[
Run a trained DenseCap model on images.

The inputs can be any one of:
- a single image: use the flag '-input_image' to give path
- a directory with images: use flag '-input_dir' to give dir path
- MSCOCO split: use flag '-input_split' to identify the split (train|val|test)

The output can be controlled with:
- max_images: maximum number of images to process. Set to -1 to process all
- output_dir: use this flag to identify directory to write outputs to
- output_vis: set to 1 to output images/json to the vis directory for nice viewing in JS/HTML
--]]


local cmd = torch.CmdLine()

-- Model options
cmd:option('-checkpoint',
  'data/models/densecap/densecap-pretrained-vgg16.t7')
cmd:option('-image_size', 720)
cmd:option('-rpn_nms_thresh', 0.7)
cmd:option('-final_nms_thresh', 0.3)
cmd:option('-num_proposals', 1000)

-- Input settings
cmd:option('-data_h5', 'data/VG-regions.h5')
cmd:option('-data_json', 'data/VG-regions-dicts.json')
cmd:option('-input_image', '',
  'A path to a single specific image to caption')
cmd:option('-input_dir', '', 'A path to a directory with images to caption')
cmd:option('-input_split', '',
  'A VisualGenome split identifier to process (train|val|test)')

-- Only used when input_split is given
cmd:option('-splits_json', 'info/densecap_splits.json')
cmd:option('-vg_img_root_dir', '', 'root directory for vg images')

-- Output settings
cmd:option('-max_images', 100, 'max number of images to process')
cmd:option('-output_dir', '')
    -- these settings are only used if output_dir is not empty
    cmd:option('-num_to_draw', 10, 'max number of predictions per image')
    cmd:option('-text_size', 2, '2 looks best I think')
    cmd:option('-box_width', 2, 'width of rendered box')
cmd:option('-output_vis', 1,
  'if 1 then writes files needed for pretty vis into vis/ ')
cmd:option('-output_vis_dir', 'vis/data')

-- Misc
cmd:option('-gpu', 0)
cmd:option('-use_cudnn', 1)
local opt = cmd:parse(arg)


function run_image(model, img_caffe, opt, dtype, label_mask)

--   -- Load, resize, and preprocess image
--   local img = image.load(img_path, 3)
--   img = image.scale(img, opt.image_size):float()
--   local H, W = img:size(2), img:size(3)
--   local img_caffe = img:view(1, 3, H, W)
--   img_caffe = img_caffe:index(2, torch.LongTensor{3, 2, 1}):mul(255)
--   local vgg_mean = torch.FloatTensor{103.939, 116.779, 123.68}
--   vgg_mean = vgg_mean:view(1, 3, 1, 1):expand(1, 3, H, W)
--   img_caffe:add(-1, vgg_mean)

  -- Run the model forward
  local boxes, scores, classes, captions, class_scores = model:forward_test(img_caffe:type(dtype), label_mask)

--   local captions = {}
--   for i = 1, boxes:size(1) do
--     table.insert(captions, 'object')
--   end
  local captions_out = nil
  if false then
	mask = classes:int():ne(1):select(2,1)
	print(mask)
	indices = torch.linspace(1, classes:size(1), classes:size(1)):long()[mask]
	print(indices)
	boxes = boxes:index(1, indices)
	scores = scores:index(1, indices)
	captions_out = {}
	for idx = 1,classes:size(1) do
		if mask[idx] ~= 0 then
		table.insert(captions_out, captions[idx])
		end
	end
	classes = classes:index(1, indices)
	print(captions_out)
  else
	captions_out = captions
  end

  local boxes_xywh
  if boxes:nDimension() ~= 0 then
    boxes_xywh = box_utils.xcycwh_to_xywh(boxes)
  else
    boxes_xywh = torch.Tensor()
  end

  local out = {
     img = img,
     boxes = boxes_xywh,
     boxes_xcycwh = boxes,
     scores = scores,
     captions = captions_out,
     classes = classes,
     class_scores = class_scores
  }

  return out
end

function result_to_json(result)
  local out = {}
  out.boxes = result.boxes:float():totable()
  if result.scores:nDimension() ~= 0 then
    out.scores = result.scores:float():view(-1):totable()
  else
    out.scores = {}
  end
  out.captions = result.captions
  return out
end

function rotate180(img)
  return image.vflip(image.hflip(img))
end

function lua_render_result(result, opt, outPath)
  -- Okay, we want to draw the bboxes onto the *original* images.
  -- Retrieve the image path from the dataset.
  local imgpath = paths.dirname(result.info.filename)
  print("Processing", imgpath)

  local rgbFile = paths.concat(imgpath, paths.basename(imgpath) .. "_color.jpg")
  local rgbIm = image.load(rgbFile)

  local crop_x1 = (result.info.ori_bbox[1] + result.info.ori_bbox[3]/2)
  local crop_x2 = (result.info.ori_bbox[1] - result.info.ori_bbox[3]/2)
  local crop_y1 = (result.info.ori_bbox[2] + result.info.ori_bbox[4]/2)
  local crop_y2 = (result.info.ori_bbox[2] - result.info.ori_bbox[4]/2)

  --local annotationIm = image.load(paths.concat(imgpath, "annotation.png"))
  --annotationIm = image.crop(annotationIm, crop_x1, crop_y1, crop_x2, crop_y2)
  image.save(paths.concat(outPath, "rgb.png"), rgbIm)

  for num_boxes=1,5 do
    -- slice results accordingly
    local slicedBoxes = result.boxes_xcycwh[{{1, num_boxes}}]:clone()

    local captions_sliced = {}
    for i = 1, num_boxes do
      table.insert(captions_sliced, "")
    end

    -- project to original image
    local scale = result.info.ori_bbox[3] / result.info.width
    slicedBoxes:mul(scale)

    slicedBoxes_xy = box_utils.xcycwh_to_xywh(slicedBoxes)

    -- render
    local outImg = vis_utils.densecap_draw(rgbIm, slicedBoxes_xy, captions_sliced, {box_width=4})

    -- save
    image.save(paths.concat(outPath, string.format("out%d.png", num_boxes)), outImg)
  end
end

function get_input_images(opt)
  -- utility function that figures out which images we should process
  -- and fetches all the raw image paths
  local image_paths = {}
  if opt.input_image ~= '' then
    table.insert(image_paths, opt.input_image)
  elseif opt.input_dir ~= '' then
    -- iterate all files in input directory and add them to work
    for fn in paths.files(opt.input_dir) do
      if string.sub(fn, 1, 1) ~= '.' then
        local img_in_path = paths.concat(opt.input_dir, fn)
        table.insert(image_paths, img_in_path)
      end
    end
  elseif opt.input_split ~= '' then
    -- load json information that contains the splits information for VG
    local info = utils.read_json(opt.splits_json)
    local split_img_ids = info[opt.input_split] -- is a table of integer ids
    for k=1,#split_img_ids do
      local img_in_path = paths.concat(opt.vg_img_root_dir, tostring(split_img_ids[k]) .. '.jpg')
      table.insert(image_paths, img_in_path)
    end
  else
    error('one of input_image, input_dir, or input_split must be provided.')
  end
  return image_paths
end

-- Load the model, and cast to the right type
local dtype, use_cudnn = utils.setup_gpus(opt.gpu, opt.use_cudnn)
local checkpoint = torch.load(opt.checkpoint)
local model = checkpoint.model
-- local model = DetectionModel(opt)
model:convert(dtype, use_cudnn)
model:setTestArgs{
  rpn_nms_thresh = opt.rpn_nms_thresh,
  final_nms_thresh = opt.final_nms_thresh,
  num_proposals = opt.num_proposals,
}
model:evaluate()

-- local bounds = {
-- 	x_min=255, y_min=5,
-- 	x_max=255+439,
-- 	y_max=5+332
-- }
-- model.nets.localization_layer:setBounds(bounds)

opt.depth = model.opt.depth

-- initialize the data loader class
local loader = DataLoader(opt)

local counter = 0
local all_losses = {}

local results_json = {}

-- We also save VOC-style result files to evaluate the VOC score using VOC tools
local vocfiles = {}
for index, cls in pairs(loader:getLabelStrings()) do
  vocfiles[index] = io.open(string.format("/tmp/densecap_det_val_%s.txt", cls), "w")
end

while true do
	counter = counter + 1

	-- Grab a batch of data and convert it to the right dtype
	local data = {}
	local loader_kwargs = {split=2, iterate=true}
	local img, gt_boxes, gt_labels, info, _ = loader:getBatch(loader_kwargs)
	local data = {
		image = img:type(dtype),
		gt_boxes = gt_boxes:type(dtype),
		gt_labels = gt_labels:type(dtype),
	}
	info = info[1] -- Since we are only using a single image

	-- Get present labels
	local label_mask = torch.IntTensor(#loader.info.label_strings):fill(0)

	for i=1,gt_labels:size(2) do
		label_mask[gt_labels[1][i][1]] = 1
	end
	label_mask = nil

    local result
    if true then
      result = run_image(model, data.image, opt, dtype, label_mask)
    else
      -- display ground truth
      result = {}
      result.boxes = box_utils.xcycwh_to_xywh(data.gt_boxes[1])
      result.img = data.image
      result.captions = {}
      for i=1,gt_labels:size(2) do
        result.captions[i] = info.filename .. loader.info.label_strings[gt_labels[1][i][1]]
      end
      result.scores = torch.zeros(result.boxes:size(1))

      if gt_labels:size(2) > 1 then
        print("Multiple objects", info.filename)
        print(result)
      end
    end

    result.info = info

    -- Write VOC output
--     if result.scores:nDimension() ~= 0 then
--       -- the box is in scaled image coordinates, scale back to orig size
--       local scalew = info.ori_width / info.width
--       local scaleh = info.ori_height / info.height
--       print('scale:', scalew, scaleh)
--
--       for i=1,result.scores:size(1) do -- for every detection
--         local x2 = result.boxes[i][1] + result.boxes[i][3]
--         local y2 = result.boxes[i][2] + result.boxes[i][4]
--
--         for j=1,result.class_scores:size(2) do
--           vocfiles[j]:write(string.format(
--             "%s %.20f %f %f %f %f\n",
--             string.gsub(path.basename(info.filename), '_color.jpg', ''), --[[math.exp(result.scores[i][1]) *]] result.class_scores[i][j],
--             scalew * result.boxes[i][1], scaleh * result.boxes[i][2], scalew * x2, scaleh * y2
--           ))
--           vocfiles[j]:flush()
--         end
--       end
--     end

	-- handle output serialization: either to directory or for pretty html vis
	if opt.output_dir ~= '' then
        local img_out_path = paths.concat(opt.output_dir, string.format("img%03d", counter))
        paths.mkdir(img_out_path)
		lua_render_result(result, opt, img_out_path)
	end
	if opt.output_vis == 1 then
		local img_name = string.format('image%d.png', counter)
		-- save the raw image to vis/data/
		local img_out_path = paths.concat(opt.output_vis_dir, img_name)

		local img = data.image:clone():float():add(1, loader.vgg_mean:expandAs(img)):mul(1.0/255.0)
		img = img[1]:index(1, torch.LongTensor{3, 2, 1})

		image.save(img_out_path, img)
		-- keep track of the (thin) json information with all result metadata
		local result_json = result_to_json(result)
		result_json.img_name = img_name
		table.insert(results_json, result_json)
	end

	-- Break out if we have processed enough images
	if info.split_bounds[1] == info.split_bounds[2] then break end
end

-- get paths to all images we should be evaluating
-- local image_paths = get_input_images(opt)
-- local num_process = math.min(#image_paths, opt.max_images)
-- local results_json = {}
-- for k=1,num_process do
--   local img_path = image_paths[k]
--   print(string.format('%d/%d processing image %s', k, num_process, img_path))
--
--   -- Load, resize, and preprocess image
--   local img = image.load(img_path, 3)
--   img = image.scale(img, opt.image_size):float()
--   local H, W = img:size(2), img:size(3)
--   local img_caffe = img:view(1, 3, H, W)
--   img_caffe = img_caffe:index(2, torch.LongTensor{3, 2, 1}):mul(255)
--   local vgg_mean = torch.FloatTensor{103.939, 116.779, 123.68}
--   vgg_mean = vgg_mean:view(1, 3, 1, 1):expand(1, 3, H, W)
--   img_caffe:add(-1, vgg_mean)
--
--   -- run the model on the image and obtain results
--   local result = run_image(model, img_caffe, opt, dtype)
--   -- handle output serialization: either to directory or for pretty html vis
--   if opt.output_dir ~= '' then
--     result.img = img
--     local img_out = lua_render_result(result, opt)
--     local img_out_path = paths.concat(opt.output_dir, paths.basename(img_path))
--     image.save(img_out_path, img_out)
--   end
--   if opt.output_vis == 1 then
--     -- save the raw image to vis/data/
--     local img_out_path = paths.concat(opt.output_vis_dir, paths.basename(img_path))
--     image.save(img_out_path, img)
--     -- keep track of the (thin) json information with all result metadata
--     local result_json = result_to_json(result)
--     result_json.img_name = paths.basename(img_path)
--     table.insert(results_json, result_json)
--   end
-- end

if #results_json > 0 then
  -- serialize to json
  local out = {}
  out.results = results_json
  out.opt = opt
  utils.write_json(paths.concat(opt.output_vis_dir, 'results.json'), out)
end

