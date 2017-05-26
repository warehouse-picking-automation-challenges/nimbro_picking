#!/usr/bin/python

"""
Print dataset statistics.
"""

import os
import yaml

import apc

def evalFrames(frames):
	bagFrames = []
	runFrames = []

	framesPerLabel = {}
	for obj in apc.OBJECTS:
		framesPerLabel[obj] = 0

	for frame in frames:
		if os.path.exists(os.path.join(frame, 'capture.bag')):
			bagFrames.append(frame)
		else:
			runFrames.append(frame)

		if os.path.exists(os.path.join(frame, 'polygons.yaml')):
			labels = apc.readLabels(frame)
			for label in labels:
				assert label in apc.OBJECTS, "Unknown label %s" % label
				framesPerLabel[label] += 1

	print "Number of frames: '%d'" % len(frames)

	print "Bag frames: %d (annotated: %d)" % (
		len(bagFrames),
		len([b for b in bagFrames if os.path.exists(os.path.join(b, 'polygons.yaml'))])
	)
	print "Run frames: %d (annotated: %d)" % (
		len(runFrames),
		len([b for b in runFrames if os.path.exists(os.path.join(b, 'polygons.yaml'))])
	)

	print "Frames per object:"
	for obj in apc.OBJECTS:
		print " - %40s %3d frames" % (obj, framesPerLabel[obj])

	return framesPerLabel

if __name__ == "__main__":
	import sys

	results = []

	if len(sys.argv) == 2:
		frames = []
		for dirpath, dirname, filenames in os.walk(sys.argv[1]):
			if 'rgb.png' in filenames:
				frames.append(dirpath)
		results.append(evalFrames(frames))
	elif len(sys.argv) > 2:
		base_path = sys.argv[1]

		for k in range(len(sys.argv) - 2):
			frames = open(sys.argv[2+k]).read().split()
			frames = [ os.path.join(base_path, frame) for frame in frames ]
			results.append(evalFrames(frames))

	print "Relative occurences:"
	print "Object split0 split1 split2 split3 split4"
	for obj in apc.OBJECTS:
		counts = [ result[obj] for result in results ]
		print "%40s" % obj, ' '.join([ "%3.3f" % (float(count) / sum(counts)) for count in counts ])

	print "LD-Score (see Iterative Stratified Sampling paper)"
	all_labels = sum([ sum(result.values()) for result in results ])
	print "Number of labels in total:", all_labels

	ld_score = 0.0
	for obj in apc.OBJECTS:
		# Ratio of positive to negative samples in whole dataset
		positives = sum([ result[obj] for result in results ])
		ratio_all = float(positives) / (all_labels - positives)
		print "Object %s: ratio %f" % (obj, ratio_all)

		mean_ratio = 0.0

		for split in results:
			# Same for only this split
			ratio = float(split[obj]) / (sum(split.values()) - split[obj])
			mean_ratio += abs(ratio - ratio_all)

		ld_score += mean_ratio / len(results)

	ld_score /= len(apc.OBJECTS)
	print "LD-score: %f" % ld_score
