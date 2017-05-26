#!/usr/bin/python
# kate: replace-tabs off; indent-width 4;

import sys
import os
import re

DATASETS = [
        '',
	'-depth',
	'-gupta',
	'-highres-gupta',
	'-hd-gupta',
	'-crop-1470-raw',
	'-crop-1080-raw',
	'-crop-720-raw',
]

TABLE = [
	'crop-720-raw',
        'crop-1080-raw',
        'crop-1470-raw',
]

VARIANTS = [
	'',
	'-cross',
	'-cluster',
	'-crosscluster',
]

OBJECTS = [
	"[background]",                            # 1
	"clamp",
	"door_handle",
	"driller",
	"extension_box",
	"stapler",
	"wrench",
]

#FIXED_RESULTS = [
	#('merged', 'svm_plain', 0, 0, 65.4),
	#('merged', 'svm_tailor', 0, 0, 66.1),
#]

def parse(split_prefix, dataset, variant):
	scores = {}

	# 40 (  3 samples): precision 0.928, recall 0.806, F-Score: 0.863
	ap_regex = re.compile(r'\s+(\d+): (\d+(\.\d+)?)')
	f1_regex = re.compile(r'\s+(?P<id>\d+) \(.*\): precision \d+(\.\d+)?, recall \d+(\.\d+)?, F-Score: (?P<f1>\d+(\.\d+)?)')

	mAP = []
	F1 = []
	ap_per_object = {}
	f1_per_object = {}

	for split in range(5):
		resultname = 'data/centauro%s-split-%ssplit0%d.txt%s.result' % (
			dataset, split_prefix, split, variant
		)
		if not os.path.exists(resultname):
			return

		results = [ line for line in open(resultname) ]

		split_mAP = None
		split_f1 = None
		for line in results:
			if line.startswith('mAP:'):
				split_mAP = float(line[4:])
			if line.startswith('Mean F score:'):
				split_f1 = 100.0 * float(line[13:])

			m = re.match(ap_regex, line)
			if m:
				classId = int(m.group(1))
				ap = float(m.group(2))
				ap_per_object.setdefault(classId, []).append(ap)

			m = re.match(f1_regex, line)
			if m:
				classId = int(m.group('id'))
				f1 = float(m.group('f1'))
				f1_per_object.setdefault(classId, []).append(f1)

		if not split_mAP or not split_f1:
			return

		mAP.append(split_mAP)
		F1.append(split_f1)

	mean_mAP = sum(mAP) / len(mAP)
	mean_F1 = sum(F1) / len(F1)
	scores['mAP'] = mean_mAP
	scores['F1'] = mean_F1
	scores['ap_breakdown'] = { id: sum(ap)/len(ap) for id,ap in ap_per_object.items() }
	scores['f1_breakdown'] = { id: sum(f1)/len(f1) for id,f1 in f1_per_object.items() }

	if variant == '':
		variant = '-normal'
	if dataset == '':
		dataset = '-rgb'
	return dataset[1:], variant[1:], scores['mAP'], scores['F1'], scores['ap_breakdown'], scores['f1_breakdown']

if __name__ == "__main__":
	split_prefix = ""
	if len(sys.argv) > 1:
		split_prefix = sys.argv[1]

	tuples = []

	for dataset in DATASETS:
		for variant in VARIANTS:
			ret = parse(split_prefix, dataset, variant)
			if ret:
				tuples.append(ret)

	tuples.sort(key=lambda x: x[2])

	print "Dataset Variant mAP F1"
	for line in tuples:
		print ' '.join([ str(l) for l in line[:4] ])

	print
	print "AP breakdown:"
	print "Class Name mAP F1"

	classes = set(line[4].keys() + line[5].keys())
	for cls in classes:
		mAP = line[4].get(cls, 0.0)
		f1 = line[5].get(cls, 0.0)
		print "%2d %40s %f %f" % (cls, OBJECTS[cls-1], mAP, f1)
	
	print ""
	print "AP/F1 comp table:"
	print ""
	tuples.sort(key=lambda x: -x[2])

	print "Dataset",
	for cls in classes:
		print "& \multicolumn{2}{c}{%s}" % OBJECTS[cls-1],
	print "& \multicolumn{2}{c}{Mean} \\\\"

	for dataset in TABLE:
		line = next(x for x in tuples if x[0] == dataset)

		print line[0],

		avg_map = 0.0
		avg_f1 = 0.0
		for cls in classes:
			mAP = line[4].get(cls, 0.0)
	                f1 = line[5].get(cls, 0.0)
			print "& %.3f & %.3f" % (mAP, f1),
			avg_map += mAP
			avg_f1 += f1

		avg_map /= len(classes)
		avg_f1 /= len(classes)
		print "& %.3f & %.3f \\\\" % (avg_map, avg_f1)

