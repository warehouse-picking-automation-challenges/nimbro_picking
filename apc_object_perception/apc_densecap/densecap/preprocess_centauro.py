# coding=utf8

import argparse, os, json, string
from collections import Counter
from Queue import Queue
from threading import Thread, Lock

from math import floor
import h5py
import numpy as np
import sys
import yaml
import os
from scipy.misc import imread, imresize
from scipy.cluster.vq import vq, kmeans, whiten

OBJECTS = [
	"[background]",
	"clamp",
	"door_handle",
	"driller",
	"extension_box",
	"stapler",
	"wrench",
]

TEST_SET = '/07/'

CROP_X = 187
CROP_Y = 0
CROP_WIDTH = 1470
CROP_HEIGHT = 1035

"""

The output HDF5 file has the following format to describe N images with
M total regions:

- images: uint8 array of shape (N, 3, image_size, image_size) of pixel data,
  in BDHW format. Images will be resized so their longest edge is image_size
  pixels long, aligned to the upper left corner, and padded with zeros.
  The actual size of each image is stored in the image_heights and image_widths
  fields.
- image_heights: int32 array of shape (N,) giving the height of each image.
- image_widths: int32 array of shape (N,) giving the width of each image.
- original_heights: int32 array of shape (N,) giving the original height of
  each image.
- original_widths: int32 array of shape (N,) giving the original width of
  each image.
- boxes: int32 array of shape (M, 4) giving the coordinates of each bounding box.
  Each row is (xc, yc, w, h) where yc and xc are center coordinates of the box,
  and are one-indexed.
- lengths: int32 array of shape (M,) giving lengths of label sequence for each box
- img_to_first_box: int32 array of shape (N,). If img_to_first_box[i] = j then
  captions[j] and boxes[j] give the first annotation for image i
  (using one-indexing).
- img_to_last_box: int32 array of shape (N,). If img_to_last_box[i] = j then
  captions[j] and boxes[j] give the last annotation for image i
  (using one-indexing).
- box_to_img: int32 array of shape (M,). If box_to_img[i] = j then then
  regions[i] and captions[i] refer to images[j] (using one-indexing).
"""

class Rectangle:
  def __init__(self, xc, yc, w, h):
    self.xc, self.yc, self.w, self.h = float(xc), float(yc), float(w), float(h)

  def area(self):
    return self.w*self.h

  def left(self):
    return self.xc - 0.5*self.w

  def right(self):
    return self.xc + 0.5*self.w

  def top(self):
    return self.yc - 0.5*self.h

  def bottom(self):
    return self.yc + 0.5*self.h

  def topLeft(self):
    return (self.left(), self.top())

  def topRight(self):
    return (self.right(), self.top())

  def bottomLeft(self):
    return (self.left(), self.bottom())

  def bottomRight(self):
    return (self.right(), self.bottom())

  def intersection(self, other):
    left   = max(self.left(), other.left())
    right  = min(self.right(), other.right())
    top    = max(self.top(), other.top())
    bottom = min(self.bottom(), other.bottom())

    if left >= right or top >= bottom:
      return Rectangle(0, 0, 0, 0)

    return Rectangle(
      xc=(left+right)/2,
      yc=(top+bottom)/2,
      w=(right-left),
      h=(bottom-top)
    )

  def iou(self, other):
    intersection = self.intersection(other).area()
    if intersection == 0:
      return 0

    assert intersection >= 0
    assert intersection <= self.area() + 1e-7
    assert intersection <= other.area() + 1e-7

    return intersection / (self.area() + other.area() - intersection)

  def unionRect(self, other):
    left = min(self.left(), other.left())
    right = max(self.right(), other.right())
    top = min(self.top(), other.top())
    bottom = max(self.bottom(), other.bottom())

    return Rectangle(
      xc=(left+right)/2,
      yc=(top+bottom)/2,
      w=(right-left),
      h=(bottom-top)
    )

  def __repr__(self):
    return u"[%dx%d+%d+%d] (+%d+%d)" % (self.w, self.h, self.xc, self.yc, self.xc - 0.5*self.w, self.yc - 0.5*self.h)

def load_gt_rectangles(yamlPath):
  gt_polygons = yaml.load(open(yamlPath))
  gt_rectangles = {}

  for item in gt_polygons['polygons']:
    x = [ point[0] for point in item['points'] ]
    y = [ point[1] for point in item['points'] ]

    bbox = Rectangle(
      xc = 0.5*(max(x) + min(x)),
      yc = 0.5*(max(y) + min(y)),
      w = (max(x) - min(x)),
      h = (max(y) - min(y))
    )

    if item['name'] == 'ground' or item['name'] == 'ignore':
      continue

    gt_rectangles.setdefault(item['name'], []).append(bbox)

  ## Merge intersecting bboxes
  # No: in the centauro dataset, we have a 1:1 correspondence between polygons and objects.
  #for name, bboxes in gt_rectangles.items():
    #for j in range(len(bboxes)-1, -1, -1):
      #for k in range(j-1, -1, -1):
        #if bboxes[j].intersection(bboxes[k]).area() > 0.0:
          #bboxes[k] = bboxes[k].unionRect(bboxes[j])
          #del bboxes[j]
          #break

  return gt_rectangles

def encode_boxes(data, original_heights, original_widths, image_size, filename_to_idx):
  all_boxes = []
  labels = []
  xwasbad = 0
  ywasbad = 0
  wwasbad = 0
  hwasbad = 0
  filename_idxs = []

  img_idx = 1
  box_idx = 1
  num_images = len(data)
  img_to_first_box = np.zeros(num_images, dtype=np.int32)
  img_to_last_box = np.zeros(num_images, dtype=np.int32)
  gt_boxes = np.zeros((num_images, 4), dtype=np.int32)
  box_to_img_python = []

  for i, path in enumerate(data):
    H, W = original_heights[i], original_widths[i]
    scale = float(image_size) / max(H, W)

    img_to_first_box[img_idx - 1] = box_idx

    gt_rectangles = load_gt_rectangles(os.path.join(path, 'polygons.yaml'))

    gt_boxes[i][0] = CROP_X + CROP_WIDTH/2
    gt_boxes[i][1] = CROP_Y + CROP_HEIGHT/2
    gt_boxes[i][2] = CROP_WIDTH
    gt_boxes[i][3] = CROP_HEIGHT

    framenum = path.split('/')[-1]
    filename_idx = filename_to_idx[os.path.join(path, '%s_color.jpg' % framenum)]

    for object_class, bboxes in gt_rectangles.items():
      for bbox in bboxes:
        # recall: x,y are 1-indexed
        x, y = round(scale*(bbox.left()-1-CROP_X)+1), round(scale*(bbox.top()-1-CROP_Y)+1)
        w, h = round(scale*bbox.w), round(scale*bbox.h)

        # clamp to image
        if x < 1: x = 1
        if y < 1: y = 1
        if x > image_size - 1:
          x = image_size - 1
          xwasbad += 1
        if y > image_size - 1:
          y = image_size - 1
          ywasbad += 1
        if x + w > image_size:
          w = image_size - x
          wwasbad += 1
        if y + h > image_size:
          h = image_size - y
          hwasbad += 1

        box = np.asarray([x+floor(w/2), y+floor(h/2), w, h], dtype=np.int32) # also convert to center-coord oriented
        assert box[2]>=0 # width height should be positive numbers
        assert box[3]>=0
        all_boxes.append(box)

        idx = OBJECTS.index(object_class)
        labels.append(idx + 1) # TH is 1-based

        box_idx += 1
        filename_idxs.append(filename_idx)
        box_to_img_python.append(img_idx-1)

    if len(gt_rectangles) == 0:
      print "img '%s' has no boxes!" % path

    img_to_last_box[img_idx - 1] = box_idx - 1 # -1 to make these inclusive limits
    img_idx += 1

  print 'number of bad x,y,w,h: ', xwasbad, ywasbad, wwasbad, hwasbad
  print 'total boxes: ', len(all_boxes)
  return np.vstack(all_boxes), img_to_first_box, img_to_last_box, np.asarray(filename_idxs, dtype=np.int32), labels, gt_boxes, box_to_img_python

def build_filename_dict(data):
  next_idx = 1
  filename_to_idx, idx_to_filename = {}, {}
  for img in data:
    framenum = img.split('/')[-1]
    filename = os.path.join(img, '%s_color.jpg' % framenum)
    filename_to_idx[filename] = next_idx
    idx_to_filename[next_idx] = filename
    next_idx += 1
  return filename_to_idx, idx_to_filename

#def encode_filenames(data, filename_to_idx):
  #filename_idxs = []
  #for img in data:
    #filename = '%d.jpg' % img['id']
    #idx = filename_to_idx[filename]
    #for region in img['regions']:
      #if region['tokens'] is None: continue
      #filename_idxs.append(idx)
  #return np.asarray(filename_idxs, dtype=np.int32)

def add_images(data, h5_file, args):
  num_images = len(data)

  num_channels = 3
  if args.depth != '':
    num_channels = 6

  print "num_channels:", num_channels
  
  shape = (num_images, num_channels, args.image_size, args.image_size)
  image_dset = h5_file.create_dataset('images', shape, dtype=np.uint8)
  original_heights = np.zeros(num_images, dtype=np.int32)
  original_widths = np.zeros(num_images, dtype=np.int32)
  image_heights = np.zeros(num_images, dtype=np.int32)
  image_widths = np.zeros(num_images, dtype=np.int32)

  # list because python 2.x is demented
  # http://stackoverflow.com/questions/4851463/python-closure-write-to-variable-in-parent-scope
  hha_mean = [np.zeros(3, dtype=np.float32)]
  
  lock = Lock()
  q = Queue()
  
  for i, img in enumerate(data):
    q.put((i, img))

  def worker():
    while True:
      i, filename = q.get()

      gt_rectangles = load_gt_rectangles(os.path.join(filename, 'polygons.yaml'))

      framenum = filename.split('/')[-1]
      img = imread(os.path.join(filename, '%s_color.jpg' % framenum))
      # handle grayscale
      if img.ndim == 2:
        img = img[:, :, None][:, :, [0, 0, 0]]

      # cut box
      img = img[CROP_Y:CROP_Y+CROP_HEIGHT, CROP_X:CROP_X+CROP_WIDTH]

      H0, W0 = img.shape[0], img.shape[1]

      img = imresize(img, float(args.image_size) / max(H0, W0))
      H, W = img.shape[0], img.shape[1]
      # swap rgb to bgr. Is this the best way?
      r = img[:,:,0].copy()
      img[:,:,0] = img[:,:,2]
      img[:,:,2] = r

      lock.acquire()
      if i % 1000 == 0:
        print 'Writing image %d / %d' % (i, len(data))
      original_heights[i] = H0
      original_widths[i] = W0
      image_heights[i] = H
      image_widths[i] = W

      if args.depth != '':
        depth = args.depth
        if '%s' in depth:
          depth = depth % framenum

        hha = imread(os.path.join(filename, depth))
        # handle grayscale
        if hha.ndim == 2:
          hha = hha[:, :, None][:, :, [0, 0, 0]]
	
	# cut box
	hha = hha[CROP_Y:CROP_Y+CROP_HEIGHT, CROP_X:CROP_X+CROP_WIDTH]

        hha = imresize(hha, float(args.image_size) / max(H0, W0))

        hha_mean[0] += hha.mean((0,1))

        image_dset[i, :, :H, :W] = np.concatenate(
          (img.transpose(2, 0, 1), hha.transpose(2, 0, 1)),
          axis=0
        )
      else:
        image_dset[i, :, :H, :W] = img.transpose(2, 0, 1)

      lock.release()
      q.task_done()
  
  print('adding images to hdf5.... (this might take a while)')
  for i in xrange(args.num_workers):
    t = Thread(target=worker)
    t.daemon = True
    t.start()
  q.join()

  hha_mean[0] /= num_images
  print 'HHA mean: ', hha_mean[0]

  h5_file.create_dataset('image_heights', data=image_heights)
  h5_file.create_dataset('image_widths', data=image_widths)
  h5_file.create_dataset('original_heights', data=original_heights)
  h5_file.create_dataset('original_widths', data=original_widths)

  return hha_mean[0]

def encode_splits(data, split_size, test_set_file):
  """ Encode splits as integers and return the array. """
  # lookup = {'train': 0, 'val': 1, 'test': 2}

  test_set = open(test_set_file).read().split('\n')
  test_set = [t for t in test_set if t]

  splits = np.zeros(len(data))

  # extract separate test set
  remaining_indices = []
  for i, frame in enumerate(data):
    for t in test_set:
      if t in frame:
        splits[i] = 2
        break
    else:
      remaining_indices.append(i)

  print " - Training:", len(remaining_indices)
  #print " - Validation:", split_size
  print " - Test:", len(data) - len(remaining_indices)

  remaining_split = np.concatenate([
    np.full((len(remaining_indices)), 0, dtype=int),
    #np.full((len(remaining_indices) - split_size,), 0, dtype=int),
    #np.full((split_size,), 1, dtype=int),
  ])

  perm = np.random.permutation(remaining_split)

  for idx, split in zip(remaining_indices, perm):
    splits[idx] = split

  return splits

def cluster_boxes(boxes, box_to_img_python, split):
  box_split = split[box_to_img_python]
  train_box_mask = np.equal(box_split, 0)
  train_boxes = boxes[train_box_mask]

  sizes = train_boxes[:,[2,3]].astype(float)
  whitened = whiten(sizes)
  clusters, distortion = kmeans(whitened, 12)

  # undo whitening
  clusters = clusters * np.std(sizes, 0)

  # round
  clusters = np.around(clusters).astype(int)

  return np.transpose(clusters)

def main(args):
  frames = []
  for dirpath, dirname, filenames in os.walk(args.dataset):
    if 'polygons.yaml' in filenames:
      annotation = yaml.load(open(os.path.join(dirpath, 'polygons.yaml')))
      if len(annotation['polygons']) > 0:
        frames.append(dirpath)
      else:
        print "Warning: Ignoring since empty:", dirpath

  if args.max_images > 0:
    frames = frames[:args.max_images]

  print "Got %d frames." % len(frames)
  split_size = int(round(args.test_split * len(frames)))

  # create the output hdf5 file handle
  f = h5py.File(args.h5_output, 'w')

  filename_to_idx, idx_to_filename = build_filename_dict(frames)

  # add several fields to the file: images, and the original/resized widths/heights
  hha_mean = add_images(frames, f, args)

  # add split information
  split = encode_splits(frames, split_size, args.test_set)
  f.create_dataset('split', data=split)

  # encode boxes
  original_heights = np.asarray(f['original_heights'])
  original_widths = np.asarray(f['original_widths'])
  boxes_matrix, img_to_first_box, img_to_last_box, box_to_img, labels, gt_boxes, box_to_img_python = encode_boxes(frames, original_heights, original_widths, args.image_size, filename_to_idx)
  f.create_dataset('boxes', data=boxes_matrix)

  # Cluster boxes to get an idea for possible anchors
  f.create_dataset('cluster_anchors', data=cluster_boxes(boxes_matrix, box_to_img_python, split))

  # integer mapping between image ids and box ids
  f.create_dataset('img_to_first_box', data=img_to_first_box)
  f.create_dataset('img_to_last_box', data=img_to_last_box)
  f.create_dataset('box_to_img', data=box_to_img)
  f.create_dataset('orig_bbox', data=gt_boxes)

  # label strings
  f.create_dataset('labels', data=labels)
  f.close()

  # and write the additional json file 
  json_struct = {
    'label_strings': OBJECTS,
    'filename_to_idx': filename_to_idx,
    'idx_to_filename': idx_to_filename,
    'hha_mean': hha_mean.tolist(),
  }
  with open(args.json_output, 'w') as f:
    json.dump(json_struct, f)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()

  # INPUT settings
  parser.add_argument('--dataset',
      required=True,
      help='Input dataset')

  # OUTPUT settings
  parser.add_argument('--json_output',
      default='data/centauro-dicts.json',
      help='Path to output JSON file')
  parser.add_argument('--h5_output',
      default='data/centauro.h5',
      help='Path to output HDF5 file')

  # OPTIONS
  parser.add_argument('--image_size',
      default=720, type=int,
      help='Size of longest edge of preprocessed images')
  parser.add_argument('--test_split', default=(5000.0 / 90000.0), type=float)
  parser.add_argument('--test_set', default='run_20160702_1153', type=str)
  parser.add_argument('--num_workers', default=5, type=int)
  parser.add_argument('--max_images', default=-1, type=int,
      help="Set to a positive number to limit the number of images we process")
  parser.add_argument('--depth', default='', type=str, help='Include depth (filename)')
  args = parser.parse_args()
  main(args)
