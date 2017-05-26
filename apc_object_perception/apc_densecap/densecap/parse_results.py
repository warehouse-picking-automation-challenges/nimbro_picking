#!/usr/bin/python
# kate: replace-tabs off; indent-width 4;

import sys
import os
import re

DATASETS = [
	'-merged',
	'-depth-old',
	'-depth',
	'-depth-hhav',
	'-depth-hhv',
	'-depth-hva',
	'-depth-upper-hha',
	'-depth-upper-hhav',
]

VARIANTS = [
	'',
	'-deep15',
	'-cross',
	'-hha_cnn',
	'-hha_concat',
	'-proposal',
	'-noaug'
]

OBJECTS = [
	"[background]",                            # 1
	"barkely_hide_bones",
	"cherokee_easy_tee_shirt",
	"clorox_utility_brush",
	"cloud_b_plush_bear",
	"command_hooks",
	"cool_shot_glue_sticks",
	"crayola_24_ct",
	"creativity_chenille_stems",
	"dasani_water_bottle",                     # 10
	"dove_beauty_bar",
	"dr_browns_bottle_brush",
	"easter_turtle_sippy_cup",
	"elmers_washable_no_run_school_glue",
	"expo_dry_erase_board_eraser",
	"fiskars_scissors_red",
	"fitness_gear_3lb_dumbbell",
	"folgers_classic_roast_coffee",
	"hanes_tube_socks",
	"i_am_a_bunny_book",                       # 20
	"jane_eyre_dvd",
	"kleenex_paper_towels",
	"kleenex_tissue_box",
	"kyjen_squeakin_eggs_plush_puppies",
	"laugh_out_loud_joke_book",
	"oral_b_toothbrush_green",
	"oral_b_toothbrush_red",
	"peva_shower_curtain_liner",
	"platinum_pets_dog_bowl",
	"rawlings_baseball",                       # 30
	"rolodex_jumbo_pencil_cup",
	"safety_first_outlet_plugs",
	"scotch_bubble_mailer",
	"scotch_duct_tape",
	"soft_white_lightbulb",
	"staples_index_cards",
	"ticonderoga_12_pencils",
	"up_glucose_bottle",
	"womens_knit_gloves",
	"woods_extension_cord",                    # 40
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

	for informed in [True, False]:
		informed_str = 'uninformed'
		if informed:
			informed_str = 'informed'

		mAP = []
		F1 = []
		ap_per_object = {}
		f1_per_object = {}

		for split in range(5):
			resultname = 'data/apc%s-split-%ssplit0%d.txt%s-%s.result' % (
				dataset, split_prefix, split, variant, informed_str
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
		scores[informed] = {}
		scores[informed]['mAP'] = mean_mAP
		scores[informed]['F1'] = mean_F1
		scores[informed]['ap_breakdown'] = { id: sum(ap)/len(ap) for id,ap in ap_per_object.items() }
		scores[informed]['f1_breakdown'] = { id: sum(f1)/len(f1) for id,f1 in f1_per_object.items() }

	assert(abs(scores[False]['F1'] - scores[True]['F1']) < 0.2)
	if variant == '':
		variant = '-normal'
	return dataset[1:], variant[1:], scores[False]['mAP'], scores[True]['mAP'], scores[True]['F1'], scores[False]['ap_breakdown'], scores[True]['ap_breakdown'], scores[True]['f1_breakdown']

def parseSVM(split_prefix, variant):
	scores = {}

	mAP = []
	F1 = []

	for split in range(5):
		resultname = 'data/svm-%s-%ssplit0%d.result' % (
			variant, split_prefix, split
		)
		if not os.path.exists(resultname):
			return

		results = [ line for line in open(resultname) ]

		split_mAP = None
		split_f1 = None
		for line in results:
			if line.startswith('mAP:'):
				split_mAP = 100.0 * float(line[4:])
			if line.startswith('F-Score:'):
				split_f1 = 100.0 * float(line[9:])

		if not split_mAP or not split_f1:
			print >>sys.stderr, 'ERROR in file "%s"' % resultname
			continue

		mAP.append(split_mAP)
		F1.append(split_f1)

	if len(mAP) == 0:
		return None

	mean_mAP = sum(mAP) / len(mAP)
	mean_F1 = sum(F1) / len(F1)

	return 'merged', variant, 0, mean_mAP, mean_F1

def sortscore(x):
	dataset_score = 0
	if x[0] == 'merged':
		dataset_score = 1
	elif x[0] == 'depth-hhav':
		dataset_score = 2
	else:
		dataset_score = 3
	
	return 10*dataset_score + x[3]

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

	for variant in ['tailor', 'notailor', 'tailor-noprop', 'notailor-noprop', 'tailor-prop', 'notailor-prop']:
		ret = parseSVM(split_prefix, variant)
		if ret:
			tuples.append(ret)

	tuples.sort(key=lambda x: sortscore(x))

	print "Dataset Variant uninformed_mAP informed_mAP F1"
	for line in tuples:
		print ' '.join([ str(l) for l in line[:5] ])

	print
	print "AP breakdown:"
	print "Class Name Uninformed Informed Gain F1"

	classes = set(line[5].keys() + line[6].keys() + line[7].keys())
	for cls in classes:
		uninf = line[5].get(cls, 0.0)
		inf = line[6].get(cls, 0.0)
		f1 = line[7].get(cls, 0.0)
		gain = inf - uninf
		print "%2d %40s %f %f %f %f" % (cls, OBJECTS[cls-1], uninf, inf, gain, f1)
