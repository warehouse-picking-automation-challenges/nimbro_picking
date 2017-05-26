#coding: utf8
import numpy as np
import yaml
import os
import joblib

from sklearn.svm import LinearSVC

from rectangle import Rectangle, find_closest_rectangle

try:
	import cv2
except ImportError:
	print("WARNING: OpenCV not available.")

WIDTH = 1920
HEIGHT = 1080
NUM_PROPOSALS = 1000
NUM_JOBS = 1

def roundPoint(p):
	return (int(round(p[0])), int(round(p[1])))

#DEFAULT_PARAMS = {
  #'min_iou': 0.4,
  #'svm_c': 0.01,
  #'num_top_rectangles': 4,
#}

#DEFAULT_PARAMS = {'num_top_rectangles': 2, 'svm_c': 1.562400868081531, 'min_iou': 0.2064922714926628, 'visualize': True}
#DEFAULT_PARAMS = {'num_top_rectangles': 5, 'svm_c': 0.051317354676444354, 'min_iou': 0.2654976307011421, 'visualize': True}

#DEFAULT_PARAMS = {'min_iou': 0.20588760750042442, 'num_top_rectangles': 6, 'svm_c': 1.25, 'visualize': False, 'tailor': False, 'balance': True}
DEFAULT_PARAMS = {'min_iou': 0.101494977043548, 'num_top_rectangles': 3.0, 'svm_c': 24.77481999167293, 'visualize': False, 'tailor': True, 'balance': True, 'simple_weighting': False, 'bbox_source': 'rgb.png.bbox.bin'}

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

		gt_rectangles.setdefault(item['name'], []).append(bbox)

	# Merge intersecting bboxes
	for name, bboxes in gt_rectangles.items():
		for j in range(len(bboxes)-1, -1, -1):
			for k in range(j-1, -1, -1):
				if bboxes[j].intersection(bboxes[k]).area() > 0.0:
					print("Merging bbox for {}".format(name))
					bboxes[k] = bboxes[k].unionRect(bboxes[j])
					del bboxes[j]
					break

	return gt_rectangles

def loadImage(imgPath, params):
	print("Reading: '{}'".format(imgPath))

	gt_rectangles = load_gt_rectangles(os.path.join(imgPath, 'polygons.yaml'))
	del gt_rectangles['box']

	img_bboxes = np.fromfile(os.path.join(imgPath, params['bbox_source']), np.float32).reshape((NUM_PROPOSALS,4))
	img_features = np.fromfile(os.path.join(imgPath, 'rgb.png.feat.bin'), np.float32).reshape((NUM_PROPOSALS,4096))
	img_scores = np.fromfile(os.path.join(imgPath, 'rgb.png.score.bin'), np.float32).reshape((NUM_PROPOSALS,1))

	detections = {}
	bboxes = {}
	training_data = {}

	if params['visualize']:
		vis_img = cv2.imread(os.path.join(imgPath, 'rgb.png'))
		vis_img = cv2.flip(vis_img, -1)
		vis_imgs = { name: vis_img.copy() for name in gt_rectangles.keys() }

	# Rotate by 180°
	def rotPoint(p):
		return (1920 - 1 - p[0], 1080 - 1 - p[1])

	for k in range(1000):
		# Bounding boxes are in xc, yc, w, h format, but in the 180° rotated image
		bbox_data = WIDTH/720.0 * img_bboxes[k,:]

		bbox = Rectangle(WIDTH - 1 - bbox_data[0], HEIGHT - 1 - bbox_data[1], bbox_data[2], bbox_data[3])

		feature_vector = np.hstack((img_features[k,:], img_scores[k]))

		# Go through ground truth bboxes
		found_match = False
		for name, rects in gt_rectangles.items():
			for rect in rects:
				iou = bbox.iou(rect)
				if iou >= params['min_iou']:
					# Positive example!
					detections.setdefault(name, []).append((iou, feature_vector, bbox))
					found_match = True

		if not found_match:
			training_data.setdefault('negative', []).append(feature_vector)

	for name, samples in detections.items():
		samples.sort(key=lambda sample: sample[0], reverse=True)
		for k in range(min(len(samples), params['num_top_rectangles'])):
			if params['visualize']:
				bbox = samples[k][2]
				cv2.rectangle(vis_imgs[name], roundPoint(rotPoint(bbox.topLeft())), roundPoint(rotPoint(bbox.bottomRight())), (0,0,255))

			if params['simple_weighting']:
				for j in range(params['num_top_rectangles'] - k):
					training_data.setdefault(name, []).append(samples[k][1])
			else:
				training_data.setdefault(name, []).append(samples[k][1])

	if params['visualize']:
		for name, img in vis_imgs.items():
			cv2.imwrite(os.path.join(imgPath, 'densecap_training_{}.png'.format(name)), img)

	return training_data

def testImageSingleObject(imgPath, training_data, present_items, k, gt_box, params):
	target = present_items[k]
	others = [ name for name in present_items if name != target ]

	print("Training for {}".format(target))
	print("Negative examples: {}".format(others))

	if target not in training_data:
		# No positive examples!
		return []

	positive = training_data[target]
	negative = []

	if params['tailor']:
		for name, data in training_data.items():
			if name in others:
				print("Negative: {} ({} samples)".format(name, len(data)))
				negative += data

		print("Absolute negatives: {}".format(len(training_data['negative'])))
		negative += training_data['negative']
	else:
		for name, data in training_data.items():
			if name != target:
				negative += data

	print("X vstack (pos: {}, neg: {})".format(len(positive), len(negative)))
	X = np.vstack((np.array(positive), np.array(negative)))

	print ("y vstack")
	y = np.hstack((np.repeat(1, len(positive)), np.repeat(-1, len(negative))))

	if params['balance']:
		balancing = 'balanced'
	else:
		balancing = None

	clf = LinearSVC(class_weight=balancing, C=params['svm_c'])

	print("Training with {} samples".format(X.shape))
	clf.fit(X, y)

	print("Prediction")
	img_bboxes = np.fromfile(os.path.join(imgPath, params['bbox_source']), np.float32).reshape((NUM_PROPOSALS,4))
	img_features = np.fromfile(os.path.join(imgPath, 'rgb.png.feat.bin'), np.float32).reshape((NUM_PROPOSALS,4096))
	img_scores = np.fromfile(os.path.join(imgPath, 'rgb.png.score.bin'), np.float32).reshape((NUM_PROPOSALS,1))

	boxes = []

	for k in range(1000):
		# Bounding boxes are in xc, yc, w, h format, but in the 180° rotated image
		bbox_data = WIDTH/720.0 * img_bboxes[k,:]

		bbox = Rectangle(WIDTH - 1 - bbox_data[0], HEIGHT - 1 - bbox_data[1], bbox_data[2], bbox_data[3])

		# Discard bbox if it does not intersect the box enough
		if bbox.intersection(gt_box).area() / bbox.area() < 0.85:
			continue

		feature_vector = np.hstack((img_features[k,:], img_scores[k]))

		prob = clf.decision_function(feature_vector.reshape(1,-1))
		boxes.append((prob, bbox))

	boxes.sort(key=lambda pair: pair[0], reverse=True)

	return boxes[:3]

def testImage(imgPath, training_data, params):
	print()
	print("Testing on image: '{}'".format(imgPath))
	print()

	test_polygons = yaml.load(open(os.path.join(imgPath, 'polygons.yaml')))
	present_items = [ item['name'] for item in test_polygons['polygons'] if item['name'] != 'box' ]

	# remove duplicates
	present_items = list(set(present_items))

	gt_rectangles = load_gt_rectangles(os.path.join(imgPath, 'polygons.yaml'))
	gt_box = gt_rectangles['box'][0]

	# compile training data for cross validation (leave-one-out scheme)
	cv_train_data = {}
	for image, data in training_data.items():
		if image == imgPath:
			continue

		for obj, samples in data.items():
			cv_train_data[obj] = cv_train_data.get(obj, []) + samples

	detections = {}
	for k in range(len(present_items)):
		detections[present_items[k]] = testImageSingleObject(imgPath, cv_train_data, present_items, k, gt_box, params)

	#detections = { name: det for name, det in zip(present_items, joblib.Parallel(n_jobs=2, backend="threading")(joblib.delayed(testImageSingleObject)(imgPath, cv_train_data, present_items, k, gt_box, params) for k in range(len(present_items)))) }

	# Visualization
	if params['visualize']:
		COLORS = [
			(0, 0, 255),
			(0, 255, 0),
			(255, 0, 0),
			(0, 255, 255),
			(255, 0, 255),
			(255, 255, 0),
			(255, 255, 255)
		]

		img = cv2.imread(os.path.join(imgPath, 'rgb.png'))

		# Rotate by 180°
		img = cv2.flip(img, -1)
		def rotPoint(p):
			return (1920 - 1 - p[0], 1080 - 1 - p[1])

		for pair, base_color in zip(detections.items(), COLORS):
			name, boxes = pair

			if len(boxes) == 0:
				continue

			cv2.putText(img, name, roundPoint(rotPoint(boxes[0][1].topRight())), cv2.FONT_HERSHEY_SIMPLEX, 1, base_color)

			for prob, bbox in boxes[:3]:
				cv2.rectangle(img, roundPoint(rotPoint(bbox.topLeft())), roundPoint(rotPoint(bbox.bottomRight())), base_color)
				base_color = [ c/1.7 for c in base_color ]

		cv2.imwrite(os.path.join(imgPath, 'densecap.png'), img)

	# Return precision/recall per object
	precision = {}
	recall = {}

	for name in present_items:
		if len(detections[name]) == 0:
			precision[name] = 0.0
			recall[name] = 0.0
			continue

		prob, detection = detections[name][0]
		ground_truth = find_closest_rectangle(gt_rectangles[name], detection)

		intersection = detection.intersection(ground_truth)

		precision[name] = intersection.area() / detection.area()
		recall[name] = intersection.area() / ground_truth.area()

	return precision, recall

def evaluate(trainingSpace, testSpace, params=None):
	training_data = {}

	# Sanitize parameters
	params = params or DEFAULT_PARAMS
	print(params)
	params['num_top_rectangles'] = int(round(params['num_top_rectangles']))

	# Load training data
	for dirpath, dirname, filenames in os.walk(trainingSpace):
		if 'rgb.png' in filenames:
			training_data[dirpath] = loadImage(dirpath, params)

	#print("Training data per object:")
	#for name, data in training_data.items():
		#print(" - {}: {} boxes".format(name, len(data)))

	precision = {}
	recall = {}
	counts = {}
	failures = {}
	failure_paths = []

	jobs = []

	for dirpath, dirname, filenames in os.walk(testSpace):
		if 'rgb.png' in filenames:
			jobs.append(dirpath)

	results = joblib.Parallel(n_jobs=NUM_JOBS, backend='threading')(
		joblib.delayed(testImage)(dirpath, training_data, params) for dirpath in jobs
	)

	for dirpath, result in zip(jobs, results):
		frame_precision, frame_recall = result

		for name, value in frame_precision.items():
			precision[name] = precision.get(name, 0.0) + value

			if value == 0.0:
				failure_paths.append(dirpath)
				failures[name] = failures.get(name, 0) + 1

			counts[name] = counts.get(name, 0) + 1
		for name, value in frame_recall.items():
			recall[name] = recall.get(name, 0.0) + value

	precision = { name : value / counts[name] for name, value in precision.items() }
	recall = { name : value / counts[name] for name, value in recall.items() }

	print()
	print()
	print("Absolute failure cases:")
	for dirpath in failure_paths:
		print(" - %s" % dirpath)

	print()
	print()
	print("Precision/recall per object:")
	namelist = sorted(precision.keys())
	for name in namelist:
		print(" - %35s: recall: %10.6f, precision: %10.6f (%5.3f complete failure rate)" % (name, recall[name], precision[name], float(failures.get(name, 0)) / counts[name]))

	mean_prec = sum(precision.values())/len(precision)
	mean_recall = sum(recall.values())/len(recall)

	if mean_prec + mean_recall == 0:
		print("Precision and recall are both zero!")
		return 0

	f_score = 2*(mean_prec*mean_recall)/(mean_prec+mean_recall)

	print("Params: %s" % params)
	print("Mean precision:", mean_prec)
	print("Mean recall:", mean_recall)
	print("F-Score:", 2*(mean_prec*mean_recall)/(mean_prec+mean_recall))

	return f_score

if __name__ == "__main__":
	import sys
	evaluate(sys.argv[1], sys.argv[2])
