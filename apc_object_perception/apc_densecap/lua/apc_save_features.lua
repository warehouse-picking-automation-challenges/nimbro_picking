require 'torch'
require 'nn'
require 'image'

require 'densecap.DenseCapModel'
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


function fwriteFloat(tensor, file)
  if not tensor then return false end
  local n = tensor:nElement()
  local s = tensor:storage()
  return assert(file:writeFloat(s) == n)
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

function fwriteDouble(tensor, file)
  if not tensor then return false end
  local n = tensor:nElement()
  local s = tensor:storage()
  return assert(file:writeDouble(s) == n)
end

local cmd = torch.CmdLine()

-- Model options
cmd:option('-checkpoint',
  'data/models/densecap/densecap-pretrained-vgg16.t7')
cmd:option('-image_size', 720)
cmd:option('-rpn_nms_thresh', 0.7)
cmd:option('-final_nms_thresh', 0.0)
cmd:option('-num_proposals', 1000)

-- Input settings
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


function run_image(model, img_path, opt, dtype)

  -- Load, resize, and preprocess image
  local img = image.load(img_path, 3)
  local tmp = image.hflip(img)
  img = image.vflip(tmp)

  img = image.scale(img, opt.image_size):float()
  local H, W = img:size(2), img:size(3)
  local img_caffe = img:view(1, 3, H, W)
  img_caffe = img_caffe:index(2, torch.LongTensor{3, 2, 1}):mul(255)
  local vgg_mean = torch.FloatTensor{103.939, 116.779, 123.68}
  vgg_mean = vgg_mean:view(1, 3, 1, 1):expand(1, 3, H, W)
  img_caffe:add(-1, vgg_mean)

  -- Run the model forward
  local boxes, scores, captions = model:forward_test(img_caffe:type(dtype))
  local boxes_xywh = box_utils.xcycwh_to_xywh(boxes)

  local out = {
    img = img,
    boxes = boxes,
    scores = scores,
    captions = captions,
  }
  return out
end

function result_to_json(result)
  local out = {}
  out.boxes = result.boxes:float():totable()
  out.scores = result.scores:float():view(-1):totable()
  out.captions = result.captions
  return out
end

function lua_render_result(result, opt)
  -- use lua utilities to render results onto the image (without going)
  -- through the vis utilities written in JS/HTML. Kind of ugly output.

  -- respect the num_to_draw setting and slice the results appropriately
  local boxes = result.boxes
  local num_boxes = math.min(opt.num_to_draw, boxes:size(1))
  boxes = boxes[{{1, num_boxes}}]
  local captions_sliced = {}
  for i = 1, num_boxes do
    table.insert(captions_sliced, result.captions[i])
  end

  -- Convert boxes and draw output image
  local draw_opt = { text_size = opt.text_size, box_width = opt.box_width }
  local img_out = vis_utils.densecap_draw(result.img, boxes, captions_sliced, draw_opt)
  return img_out
end

function add_images(path, result)
  for subdir in paths.iterdirs(path) do
    local subpath = paths.concat(path, subdir)
    local imgname = paths.concat(subpath, "rgb.png")
    if paths.filep(imgname) then
      table.insert(result, imgname)
    else
      add_images(subpath, result)
    end
  end
end

function get_input_images(opt)
  -- utility function that figures out which images we should process 
  -- and fetches all the raw image paths
  local image_paths = {}
  
  if opt.input_dir ~= '' then
    add_images(opt.input_dir, image_paths)
  else
    error("Please specify input_dir")
  end

  return image_paths
end

-- Load the model, and cast to the right type
local dtype, use_cudnn = utils.setup_gpus(opt.gpu, opt.use_cudnn)
local checkpoint = torch.load(opt.checkpoint)
local model = checkpoint.model
model:convert(dtype, use_cudnn)
model:setTestArgs{
  rpn_nms_thresh = opt.rpn_nms_thresh,
  final_nms_thresh = opt.final_nms_thresh,
  num_proposals = opt.num_proposals,
}
model:evaluate()

print(model.net)

for k=1,18 do
  print(model.net.modules[4].forwardnodes[k].data.annotations)
end

-- get paths to all images we should be evaluating
local image_paths = get_input_images(opt)
local num_process = #image_paths
local results_json = {}
for k=1,num_process do
  local img_path = image_paths[k]
  print(string.format('%d/%d processing image %s', k, num_process, img_path))
  -- run the model on the image and obtain results
  local result = run_image(model, img_path, opt, dtype)

  -- get ROI features and bbox coordinates
  local file = torch.DiskFile(img_path .. ".bbox.bin", "w"):binary()
  fwriteFloat(model.net.modules[3].output[2]:float(), file)

  file = torch.DiskFile(img_path .. ".regbbox.bin", "w"):binary()
  fwriteFloat(result.boxes:float(), file)

  file = torch.DiskFile(img_path .. ".score.bin", "w"):binary()
  fwriteFloat(result.scores:float(), file)
  
  file = torch.DiskFile(img_path .. ".feat.bin", "w"):binary()
  fwriteFloat(model.net.modules[4].forwardnodes[6].data.input[1]:float(), file)

--   if true then
--     local bboxes = model.net.modules[3].output[2]:float():clone()
--
-- 	model.nets.localization_layer.overrideBoxes = true
-- 	model.nets.localization_layer.overrideBoxesData = bboxes:cuda()
--
-- 	local result = run_image(model, img_path, opt, dtype)
-- 	local file = torch.DiskFile(img_path .. ".bboxout.bin", "w"):binary()
-- 	fwriteFloat(model.net.modules[3].output[2]:float(), file)
--
-- 	print(bboxes[1])
-- 	print(model.net.modules[3].output[2]:float()[1])
-- 	print(bboxes[1] - model.net.modules[3].output[2]:float()[1])
--
-- 	file = torch.DiskFile(img_path .. ".refeat.bin", "w"):binary()
-- 	fwriteFloat(model.net.modules[4].forwardnodes[6].data.input[1]:float(), file)
--
-- 	model.nets.localization_layer.overrideBoxes = false
--   end

  if true then
	local hha_path = paths.dirname(image_paths[k]) .. "/feature_hha.png"

	model.nets.localization_layer.overrideBoxes = true
	model.nets.localization_layer.overrideBoxesData = model.net.modules[3].output[2]:float():cuda()

	local result = run_image(model, img_path, opt, dtype)

	file = torch.DiskFile(img_path .. ".hha.feat.bin", "w"):binary()
	fwriteFloat(model.net.modules[4].forwardnodes[6].data.input[1]:float(), file)

	model.nets.localization_layer.overrideBoxes = false
  end

--   local request = torch.DiskFile(img_path .. ".densecap_req_oral_b_toothbrush_red.bin", "r", true)
--   if request then
--     request:binary()
--     local bboxes = freadFloat(request)
--
--     if bboxes then
-- 		model.nets.localization_layer.overrideBoxes = true
-- 		model.nets.localization_layer.overrideBoxesData = bboxes:cuda()
--
-- 		local result = run_image(model, img_path, opt, dtype)
-- 		local file = torch.DiskFile(img_path .. ".req.bbox.bin", "w"):binary()
-- 		fwriteFloat(model.net.modules[3].output[2]:float(), file)
--
-- 		file = torch.DiskFile(img_path .. ".req.feat.bin", "w"):binary()
-- 		fwriteFloat(model.net.modules[4].forwardnodes[6].data.input[1]:float(), file)
--
-- 		model.nets.localization_layer.overrideBoxes = false
-- 	end
--   end

--   request = torch.DiskFile(img_path .. ".densecap_gt_req.bin", "r", true)
--   if request then
--     request:binary()
--     local bboxes = freadFloat(request)
--
-- 	if bboxes then
-- 		model.nets.localization_layer.overrideBoxes = true
-- 		model.nets.localization_layer.overrideBoxesData = bboxes:cuda()
--
-- 		local result = run_image(model, img_path, opt, dtype)
-- 		local file = torch.DiskFile(img_path .. ".densecap_gt_bbox.bin", "w"):binary()
-- 		fwriteFloat(model.net.modules[3].output[2]:float(), file)
--
-- 		file = torch.DiskFile(img_path .. ".densecap_gt_feat.bin", "w"):binary()
-- 		fwriteFloat(model.net.modules[4].forwardnodes[6].data.input[1]:float(), file)
--
-- 		model.nets.localization_layer.overrideBoxes = false
-- 	end
--   end

  request = torch.DiskFile(img_path .. ".prop.bbox.bin", "r", true)
  if request then
    request:binary()
    local bboxes = freadFloat(request)

	if bboxes then
		model.nets.localization_layer.overrideBoxes = true
		model.nets.localization_layer.overrideBoxesData = bboxes:cuda()

		local result = run_image(model, img_path, opt, dtype)
		local file = torch.DiskFile(img_path .. ".prop.bboxout.bin", "w"):binary()
		fwriteFloat(model.net.modules[3].output[2]:float(), file)

		file = torch.DiskFile(img_path .. ".prop.feat.bin", "w"):binary()
		fwriteFloat(model.net.modules[4].forwardnodes[6].data.input[1]:float(), file)

		model.nets.localization_layer.overrideBoxes = false
	end
  end
end

if #results_json > 0 then
  -- serialize to json
  local out = {}
  out.results = results_json
  out.opt = opt
  utils.write_json(paths.concat(opt.output_vis_dir, 'results.json'), out)
end
