
# coding=utf8

import argparse, os

from math import floor
import numpy as np
from sklearn import cross_validation
import sys

def encode_splits(data, split_size, test_set):
  """ Encode splits as integers and return the array. """
  # lookup = {'train': 0, 'val': 1, 'test': 2}

  splits = np.zeros(len(data))

  # extract separate test set
  remaining_indices = []
  for i, frame in enumerate(data):
    if test_set in frame:
      splits[i] = 2
    else:
      remaining_indices.append(i)

  print " - Training:", len(remaining_indices) - split_size
  print " - Validation:", split_size
  print " - Test:", len(data) - len(remaining_indices)

  remaining_split = np.concatenate([
    np.full((len(remaining_indices) - split_size,), 0, dtype=int),
    np.full((split_size,), 1, dtype=int),
  ])

  perm = np.random.permutation(remaining_split)

  for idx, split in zip(remaining_indices, perm):
    splits[idx] = split

  return splits

def main(args):
  frames = []
  for dirpath, dirname, filenames in os.walk(args.dataset):
    if 'polygons.yaml' in filenames:
      frames.append(os.path.relpath(dirpath, args.dataset))

  print "Got %d frames." % len(frames)

  # K-Fold cross validation
  kf = cross_validation.KFold(len(frames), n_folds=5, shuffle=True)
  for idx, pair in enumerate(kf):
    train_index, test_index = pair
    out = open('data/split%02d.txt' % idx, 'w')
    for idx in test_index:
      out.write(frames[idx] + '\n')


if __name__ == '__main__':
  parser = argparse.ArgumentParser()

  # INPUT settings
  parser.add_argument('--dataset',
      required=True,
      help='Input dataset')

  args = parser.parse_args()
  main(args)
