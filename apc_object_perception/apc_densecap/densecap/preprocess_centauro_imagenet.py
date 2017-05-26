# coding=utf8

import argparse, os, json, string
from collections import Counter
from Queue import Queue
from threading import Thread, Lock

from math import floor
import h5py
import numpy as np
import sys
import xmltodict
import os
from scipy.misc import imread, imresize

OBJECTS = [
	("", "[background]"),
	("n03239726", "drill"),
	("n03481172", "hammer"),
	("n02680754", "wrench"),
]

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

def encode_boxes(base_path, data, original_heights, original_widths, image_size, filename_to_idx):
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

  for i, path in enumerate(data):
    H, W = original_heights[i], original_widths[i]
    scale = float(image_size) / max(H, W)

    img_to_first_box[img_idx - 1] = box_idx

    xmlpath = os.path.join(os.path.join(base_path, "Annotations"), path + ".xml")
    xmldata = xmltodict.parse(open(xmlpath))

    xmlboxes = xmldata['annotation']['object']
    if not isinstance(xmlboxes, list):
      xmlboxes = [xmlboxes]
    else:
      print "Multiple objects:", path, len(xmlboxes)

    filename_idx = filename_to_idx[path]

    for xmlbox in xmlboxes:
      object_class = xmlbox['name']
      x1 = float(xmlbox['bndbox']['xmin'])
      x2 = float(xmlbox['bndbox']['xmax'])
      y1 = float(xmlbox['bndbox']['ymin'])
      y2 = float(xmlbox['bndbox']['ymax'])

      bbox = Rectangle((x1 + x2) / 2.0, (y1 + y2) / 2.0, x2 - x1, y2 - y1)

      # recall: x,y are 1-indexed
      x, y = round(scale*(bbox.left()-1)+1), round(scale*(bbox.top()-1)+1)
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

      idx = 0
      for i, known_obj in enumerate(OBJECTS):
        if known_obj[0] == object_class:
          idx = i
          break
      else:
        raise "Unknown object class " + object_class

      labels.append(idx + 1) # TH is 1-based

      box_idx += 1
      filename_idxs.append(filename_idx)

    if len(xmlboxes) == 0:
      print "img '%s' has no boxes!" % path

    img_to_last_box[img_idx - 1] = box_idx - 1 # -1 to make these inclusive limits
    img_idx += 1

  print 'number of bad x,y,w,h: ', xwasbad, ywasbad, wwasbad, hwasbad
  print 'total boxes: ', len(all_boxes)
  return np.vstack(all_boxes), img_to_first_box, img_to_last_box, np.asarray(filename_idxs, dtype=np.int32), labels

def build_filename_dict(data):
  next_idx = 1
  filename_to_idx, idx_to_filename = {}, {}
  for img in data:
    filename = img
    #framenum = img.split('/')[-1]
    #filename = os.path.join(img, '%s_color.jpg' % framenum)
    filename_to_idx[filename] = next_idx
    idx_to_filename[next_idx] = filename
    next_idx += 1
  return filename_to_idx, idx_to_filename

def add_images(base_path, data, h5_file, args):
  num_images = len(data)

  num_channels = 3

  shape = (num_images, num_channels, args.image_size, args.image_size)
  image_dset = h5_file.create_dataset('images', shape, dtype=np.uint8)
  original_heights = np.zeros(num_images, dtype=np.int32)
  original_widths = np.zeros(num_images, dtype=np.int32)
  image_heights = np.zeros(num_images, dtype=np.int32)
  image_widths = np.zeros(num_images, dtype=np.int32)

  lock = Lock()
  q = Queue()

  for i, img in enumerate(data):
    q.put((i, img))

  def worker():
    while True:
      i, filename = q.get()

      img = imread(os.path.join(os.path.join(base_path, "JPEGImages"), filename + ".JPEG"))

      # handle grayscale
      if img.ndim == 2:
        img = img[:, :, None][:, :, [0, 0, 0]]

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

      image_dset[i, :, :H, :W] = img.transpose(2, 0, 1)

      lock.release()
      q.task_done()

  print('adding images to hdf5.... (this might take a while)')
  for i in xrange(args.num_workers):
    t = Thread(target=worker)
    t.daemon = True
    t.start()
  q.join()

  h5_file.create_dataset('image_heights', data=image_heights)
  h5_file.create_dataset('image_widths', data=image_widths)
  h5_file.create_dataset('original_heights', data=original_heights)
  h5_file.create_dataset('original_widths', data=original_widths)

def encode_splits(data, split_size, train_cnt):
  """ Encode splits as integers and return the array. """
  # lookup = {'train': 0, 'val': 1, 'test': 2}

  print " - Training:", train_cnt - split_size
  print " - Validation:", split_size
  print " - Test:", len(data) - train_cnt

  splits = np.full(len(data), 2)

  remaining_split = np.concatenate([
    np.full((train_cnt,), 0, dtype=int), # - split_size
    #np.full((split_size,), 1, dtype=int),
  ])

  perm = np.random.permutation(remaining_split)

  for idx, split in enumerate(perm):
    splits[idx] = split

  return splits

def main(args):
  base_path = args.dataset

  train_file = os.path.join(base_path, "ImageSets/Main/train.txt")
  val_file = os.path.join(base_path, "ImageSets/Main/val.txt")

  frames = []
  train_cnt = 0
  for line in open(train_file):
    if len(line.strip()) == 0:
      continue
    frames.append(line.strip())
    train_cnt += 1

  for line in open(val_file):
    if len(line.strip()) == 0:
      continue
    frames.append(line.strip())

  if args.max_images > 0:
    frames = frames[:args.max_images]

  print "Got %d frames." % len(frames)
  split_size = int(round(args.test_split * len(frames)))

  # create the output hdf5 file handle
  f = h5py.File(args.h5_output, 'w')

  filename_to_idx, idx_to_filename = build_filename_dict(frames)

  # add several fields to the file: images, and the original/resized widths/heights
  add_images(base_path, frames, f, args)

  # add split information
  split = encode_splits(frames, split_size, train_cnt)
  f.create_dataset('split', data=split)

  # encode boxes
  original_heights = np.asarray(f['original_heights'])
  original_widths = np.asarray(f['original_widths'])
  boxes_matrix, img_to_first_box, img_to_last_box, box_to_img, labels = encode_boxes(base_path, frames, original_heights, original_widths, args.image_size, filename_to_idx)
  f.create_dataset('boxes', data=boxes_matrix)

  # integer mapping between image ids and box ids
  f.create_dataset('img_to_first_box', data=img_to_first_box)
  f.create_dataset('img_to_last_box', data=img_to_last_box)
  f.create_dataset('box_to_img', data=box_to_img)

  # label strings
  f.create_dataset('labels', data=labels)
  f.close()

  # and write the additional json file
  json_struct = {
    'label_strings':  [obj[1] for obj in OBJECTS],
    'filename_to_idx': filename_to_idx,
    'idx_to_filename': idx_to_filename,
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
      default='data/centauro-imagenet.json',
      help='Path to output JSON file')
  parser.add_argument('--h5_output',
      default='data/centauro-imagenet.h5',
      help='Path to output HDF5 file')

  # OPTIONS
  parser.add_argument('--image_size',
      default=720, type=int,
      help='Size of longest edge of preprocessed images')
  parser.add_argument('--test_split', default=(5000.0 / 90000.0), type=float)
  parser.add_argument('--num_workers', default=5, type=int)
  parser.add_argument('--max_images', default=-1, type=int,
      help="Set to a positive number to limit the number of images we process")
  args = parser.parse_args()
  main(args)

