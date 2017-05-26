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

def iterativeStratification(dataset_input, k):
	dataset = list(dataset_input)

	# Calculate the desired number of examples at each subset
	count = [ len(dataset) / k for i in range(k) ]

	# Calculate the desired number of examples of each label at each subset
	countPerLabel = []
	for obj in apc.OBJECTS:
		num_examples = len( [ d for d in dataset if obj in d.labels ] )
		countPerLabel.append([ num_examples / k for i in range(k) ])

	subsets = [ [] for i in range(k) ]

	while len(dataset) != 0:
		# Find the label with the fewest (but at least one) remaining examples,
		# breaking ties randomly

		dataset_for_labels = {}
		for example in dataset:
			for label in example.labels:
				dataset_for_labels.setdefault(label, set()).add(example)

		dataset_for_labels = dataset_for_labels.items()
		random.shuffle(dataset_for_labels)

		min_label, min_dataset = min(dataset_for_labels, key=lambda x: len(x[1]))
		min_label_idx = apc.OBJECTS.index(min_label)

		print "Processing label '%s' (%d examples)" % (min_label, len(min_dataset))

		for example in min_dataset:
			# Find the subset with the largest number of desired examples for
			# this label, breaking ties by considering the largest number of desired
			# examples, breaking further ties randomly

			M_val = max(countPerLabel[min_label_idx])
			M = [ i for i, val in enumerate(countPerLabel[min_label_idx]) if val == M_val ]
			m = None

			if len(M) == 1: # no tie-break necessary
				m = M[0]
			else:
				max_count = max([ v for i, v in enumerate(count) if i in M ])
				counts = [ i for i, v in enumerate(count) if i in M and v == max_count ]
				random.shuffle(counts)
				m = counts[0]

			subsets[m].append(example.dirpath)
			dataset.remove(example)

			for label in example.labels:
				countPerLabel[apc.OBJECTS.index(label)][m] -= 1

			count[m] -= 1

	return subsets

if __name__ == "__main__":
	import argparse
	parser = argparse.ArgumentParser(description='Iterative Stratified Sampling')

	parser.add_argument('--require-capture', action="store_true", default=False, help='Require that a capture.bag file is present')
	parser.add_argument('dataset', action='store')

	args = parser.parse_args()

	frames = []

	for dirpath, dirname, filenames in os.walk(args.dataset):
		if 'polygons.yaml' in filenames:
			if args.require_capture:
				if not 'capture.bag' in filenames:
					print 'Skipping run frame %s' % dirpath
					continue

			frames.append(dirpath)

	dataset = [ Frame(frame) for frame in frames ]

	subsets = iterativeStratification(dataset, 5)

	for i, subset in enumerate(subsets):
		subset_frames = [ os.path.relpath(frame, args.dataset) for frame in subset ]
		open('split%02d.txt' % i, 'w').write('\n'.join(subset_frames) + '\n')
