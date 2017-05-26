#!/usr/bin/python
# coding: utf8

"""
Iterative stratified sampling for multi-label datasets

following

Sechidis, Konstantinos; Tsoumakas, Grigorios; Vlahavas, Ioannis (2011)
On the stratification of multi-label data. ECML PKDD. pp. 145–158.
"""

import apc
import os
import random

class Frame:
	def __init__(self, dirpath):
		self.dirpath = dirpath
		self.labels = apc.readLabels(dirpath)

	def __hash__(self):
		return hash(self.dirpath)

def slice_list(input, size):
	input_size = len(input)
	slice_size = input_size / size
	remain = input_size % size
	result = []
	iterator = iter(input)
	for i in range(size):
		result.append([])
		for j in range(slice_size):
			result[i].append(iterator.next())
		if remain:
			result[i].append(iterator.next())
			remain -= 1
	return result

def randomSampling(dataset_input, k):
	dataset = list(dataset_input)

	# Just shuffle the dataset and divide into k splits
	random.shuffle(dataset)
	dirpaths = [ frame.dirpath for frame in dataset ]

	return slice_list(dirpaths, k)

if __name__ == "__main__":
	import sys

	frames = []

	for dirpath, dirname, filenames in os.walk(sys.argv[1]):
		if 'polygons.yaml' in filenames:
			frames.append(dirpath)

	dataset = [ Frame(frame) for frame in frames ]

	subsets = randomSampling(dataset, 5)

	for i, subset in enumerate(subsets):
		subset_frames = [ os.path.relpath(frame, sys.argv[1]) for frame in subset ]
		open('split%02d.txt' % i, 'w').write('\n'.join(subset_frames) + '\n')

