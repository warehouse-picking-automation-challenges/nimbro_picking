
require('image')
local box_utils = require 'densecap.box_utils'

-- Try to load image_augmentation
local have_image_augmentation, _ = pcall(require, 'image_augmentation')

if not have_image_augmentation then
  print('WARNING: Using slow LUA data augmentation')
else
  image_augmentation.setSeed(123) -- TODO: Connect to opt.seed
end

local augmentation = {}

-- taken from https://github.com/brainstorm-ai/DIGITS/blob/master/tools/torch/data.lua
-- (NVIDIA DIGITS)

-- HSV Augmentation
-- Parameters:
-- @param im (tensor): input image
-- @param augHSV (table): standard deviations under {H,S,V} keys with float values.
function augmentation.augmentHSV(im_rgb, augHSV)
  -- Fair augHSV standard deviation values are {H=0.02,S=0.04,V=0.08}

  -- swap RGB
  local rgb = im_rgb:index(1, torch.LongTensor{3,2,1}) / 255.0

  local im_hsv = image.rgb2hsv(rgb)
  if augHSV.H >0 then
    -- We do not need to account for overflow because every number wraps around (1.1=2.1=3.1,etc)
    -- We add a round value (+ 1) to prevent an underflow bug (<0 becomes glitchy)
    im_hsv[1] = im_hsv[1]+(1 + torch.normal(0, augHSV.S))
  end
  if augHSV.S >0 then
    im_hsv[2] = im_hsv[2]+torch.normal(0, augHSV.S)
    im_hsv[2].image.saturate(im_hsv[2]) -- bound saturation between 0 and 1
  end
  if augHSV.V >0 then
    im_hsv[3] = im_hsv[3]+torch.normal(0, augHSV.V)
    im_hsv[3].image.saturate(im_hsv[3]) -- bound value between 0 and 1
  end
  return 255.0 * image.hsv2rgb(im_hsv):index(1, torch.LongTensor{3,2,1})
end

function augmentation.warpPoint(x,y, flow)
  local x_ = flow[1][y][x]
  local y_ = flow[2][y][x]

  x_ = math.max(math.min(flow:size(3), x_), 1)
  y_ = math.max(math.min(flow:size(2), y_), 1)

  return x_, y_
end

-- Scale and Rotation augmentation (warping)
-- Parameters:
-- @param im (tensor): input image
-- @param augRot (float): extremes of random rotation, uniformly distributed between
-- @param augScale (float): the standard deviation of the extra scaling factor
function augmentation.scale(im, gt_rectangles)
  local width = im:size()[3]
  local height = im:size()[2]

  -- Scale <0=zoom in(+rand crop), >0=zoom out

  local scale_x = torch.normal(0, 0.05) -- normal distribution
  -- Given a zoom in or out, we move around our canvas.
  local scale_y = scale_x -- keep aspect ratio the same
  local move_x = torch.uniform(-scale_x, scale_x)
  local move_y = torch.uniform(-scale_y, scale_y)

  -- x/y grids
  local grid_x = torch.ger( torch.ones(height), torch.linspace(-1-scale_x,1+scale_x,width) )
  local grid_y = torch.ger( torch.linspace(-1-scale_y,1+scale_y,height), torch.ones(width) )

  local flow = torch.FloatTensor()
  flow:resize(2,height,width)
  flow:zero()

  -- Apply scale
  flow_scale = torch.FloatTensor()
  flow_scale:resize(2,height,width)
  flow_scale[1] = grid_y
  flow_scale[2] = grid_x
  flow_scale[1]:add(1+move_y):mul(0.5) -- move ~[-1 1] to ~[0 1]
  flow_scale[2]:add(1+move_x):mul(0.5) -- move ~[-1 1] to ~[0 1]
  flow_scale[1]:mul(height-1)
  flow_scale[2]:mul(width-1)
  flow:add(flow_scale)

  local warped_img = image.warp(im, flow, 'bilinear', false)

  local warpPoint = function(x,y)
    local dest_x = (x - 0.5 * (move_x - scale_x) * (width-1)) / (1 + scale_x) + 1
    local dest_y = (y - 0.5 * (move_y - scale_y) * (height-1)) / (1 + scale_y) + 1

    dest_x = math.max(math.min(width, dest_x), 1)
    dest_y = math.max(math.min(height, dest_y), 1)

    return dest_x, dest_y
  end

  -- Also warp bounding boxes
  local gt_rectangles_x1y1x2y2 = box_utils.xcycwh_to_x1y1x2y2(gt_rectangles)
  for k=1,gt_rectangles:size(1) do
    local x1, y1 = warpPoint(gt_rectangles_x1y1x2y2[k][1], gt_rectangles_x1y1x2y2[k][2])
    local x2, y2 = warpPoint(gt_rectangles_x1y1x2y2[k][3], gt_rectangles_x1y1x2y2[k][4])
    gt_rectangles_x1y1x2y2[k][1] = x1
    gt_rectangles_x1y1x2y2[k][2] = y1
    gt_rectangles_x1y1x2y2[k][3] = x2
    gt_rectangles_x1y1x2y2[k][4] = y2
  end

  return warped_img, box_utils.x1y1x2y2_to_xcycwh(gt_rectangles_x1y1x2y2)
end

function augmentation.augment(img, gt_boxes, gt_labels)
  local width = img:size()[3]
  local height = img:size()[2]

  -- Try to use C impl if possible (fast path)
  if have_image_augmentation then
    local transformer = image_augmentation.Transformer({
      mirrorH = true,
      mirrorV = false,
      HSV = {
        H = 0.00,
        S = 0.00,
        V = 0.00
      },
      scale = 0.0
    })

    local warpedImg = img:clone() / 255.0
    transformer:transformImage(warpedImg[{{1,3}}])

    -- Are we using depth?
    if warpedImg:size(1) > 3 then
      -- Transform HHA as well, but do not attempt HSV augmentation
      transformer:transformImage(warpedImg[{{4,6}}], false)
    end
    warpedImg:mul(255.0)

    local warpedBoxes = transformer:transformRects_xcycwh(width, height, gt_boxes:float()):int()
    warpedBoxes, validBoxes = box_utils.clip_boxes(warpedBoxes, {
      x_min=1,y_min=1,x_max=width-1,y_max=height-1
    }, 'xcycwh')

    -- discard invalid (out of img) boxes
    indices = torch.linspace(1, gt_boxes:size(1), gt_boxes:size(1)):long()
    indices = indices[validBoxes]

    warpedBoxes = warpedBoxes:index(1, indices)
    gt_labels = gt_labels:index(1, indices)

    return warpedImg, warpedBoxes, gt_labels
  end

  -- slow path

  if torch.bernoulli() == 1 then
    img = image.hflip(img)
    -- also flip annotationss
    gt_boxes[{ {}, 1 }]  = width - gt_boxes[{ {}, 1 }]
  end
  if torch.bernoulli() == 1 then
    img = image.vflip(img)
    -- also flip annotations
    gt_boxes[{ {}, 2 }]  = height - gt_boxes[{ {}, 2 }]
  end

  -- HSV augmentation
  img = augmentation.augmentHSV(img, {H=0.02, S=0.04, V=0.08})

  -- Scaling
  img, gt_boxes = augmentation.scale(img, gt_boxes)

  return img, gt_boxes, gt_labels
end

return augmentation

