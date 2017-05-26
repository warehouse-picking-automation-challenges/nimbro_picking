#coding: utf8
import numpy as np
import yaml
import os
import joblib

from sklearn.svm import LinearSVC

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

def roundPoint(p):
  return (int(round(p[0])), int(round(p[1])))

WIDTH = 1920
HEIGHT = 1080

training_data = {}

def find_closest_rectangle(rectangles, key):
  minDist = None
  minRect = None

  for rect in rectangles:
    dist = (key.xc - rect.xc)**2 + (key.yc - rect.yc)**2
    if not minDist or dist < minDist:
      minDist = dist
      minRect = rect
  
  return minRect

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
	  print "Merging bbox for", name
	  bboxes[k] = bboxes[k].unionRect(bboxes[j])
	  del bboxes[j]
	  break

  return gt_rectangles

def loadImage(imgPath):
  print "Reading: '%s'" % imgPath

  gt_rectangles = load_gt_rectangles(os.path.join(imgPath, 'polygons.yaml'))
  del gt_rectangles['box']

  img_bboxes = np.fromfile(os.path.join(imgPath, 'rgb.png.bbox.bin'), np.float32).reshape((1000,4))
  img_features = np.fromfile(os.path.join(imgPath, 'rgb.png.feat.bin'), np.float32).reshape((1000,4096))
  
  detections = {}

  for k in range(1000):
    # Bounding boxes are in xc, yc, w, h format, but in the 180° rotated image
    bbox_data = WIDTH/720.0 * img_bboxes[k,:]
    
    bbox = Rectangle(WIDTH - 1 - bbox_data[0], HEIGHT - 1 - bbox_data[1], bbox_data[2], bbox_data[3])

    # Go through ground truth bboxes
    found_match = False
    for name, rects in gt_rectangles.items():
      for rect in rects:
	iou = bbox.iou(rect)
	if iou >= 0.4:
	  # Positive example!
	  detections.setdefault(name, []).append((iou, img_features[k,:]))
	  found_match = True
    
    if not found_match:
      training_data.setdefault('negative', []).append(img_features[k,:])
  
  for name, samples in detections.items():
    samples.sort(key=lambda sample: sample[0], reverse=True)
    for k in range(min(len(samples), 4)):
      training_data.setdefault(name, []).append(samples[k][1])


def testImageSingleObject(imgPath, training_data, present_items, k, gt_box):
  target = present_items[k]
  others = [ name for name in present_items if name != target ]

  print "Training for %s" % target
  print "Negative examples: %s" % others

  positive = training_data[target]
  negative = []
  
  for name, data in training_data.items():
    if name in others:
      negative += data
  
  negative += training_data['negative']

  X = np.vstack((np.array(positive), np.array(negative)))
  y = np.hstack((np.repeat(1, len(positive)), np.repeat(-1, len(negative))))
  
  clf = LinearSVC(class_weight='auto', C=0.01)
  
  print "Training"
  clf.fit(X, y)

  print "Prediction"
  img_bboxes = np.fromfile(os.path.join(imgPath, 'rgb.png.bbox.bin'), np.float32).reshape((1000,4))
  img_features = np.fromfile(os.path.join(imgPath, 'rgb.png.feat.bin'), np.float32).reshape((1000,4096))
  
  boxes = []
  
  for k in range(1000):
    # Bounding boxes are in xc, yc, w, h format, but in the 180° rotated image
    bbox_data = WIDTH/720.0 * img_bboxes[k,:]
    
    bbox = Rectangle(WIDTH - 1 - bbox_data[0], HEIGHT - 1 - bbox_data[1], bbox_data[2], bbox_data[3])
    
    # Discard bbox if it does not intersect the box enough
    if bbox.intersection(gt_box).area() / bbox.area() < 0.6:
      continue

    prob = clf.decision_function(img_features[k,:])
    boxes.append((prob, bbox))
  
  boxes.sort(key=lambda pair: pair[0], reverse=True)
  
  return boxes[:3]

def testImage(imgPath):
  print
  print "Testing on image: '%s'" % imgPath
  print
  
  test_polygons = yaml.load(open(os.path.join(imgPath, 'polygons.yaml')))
  present_items = [ item['name'] for item in test_polygons['polygons'] if item['name'] != 'box' ]
  
  # remove duplicates
  present_items = list(set(present_items))
  
  gt_rectangles = load_gt_rectangles(os.path.join(imgPath, 'polygons.yaml'))
  gt_box = gt_rectangles['box'][0]

  detections = { name: det for name, det in zip(present_items, joblib.Parallel(n_jobs=8, backend="threading")(joblib.delayed(testImageSingleObject)(imgPath, training_data, present_items, k, gt_box) for k in range(len(present_items)))) }

  # Visualization
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
  for pair, base_color in zip(detections.items(), COLORS):
    name, boxes = pair
    
    cv2.putText(img, name, roundPoint(boxes[0][1].bottomLeft()), cv2.FONT_HERSHEY_SIMPLEX, 1, base_color)
    
    for prob, bbox in boxes[:2]:
      cv2.rectangle(img, roundPoint(bbox.topLeft()), roundPoint(bbox.bottomRight()), base_color)
      base_color = [ c/1.7 for c in base_color ]

  cv2.imwrite(os.path.join(imgPath, 'densecap.png'), img)
  
  # Return precision/recall per object
  precision = {}
  recall = {}
  
  for name in present_items:
    prob, detection = detections[name][0]
    ground_truth = find_closest_rectangle(gt_rectangles[name], detection)
    
    intersection = detection.intersection(ground_truth)
    
    precision[name] = intersection.area() / detection.area()
    recall[name] = intersection.area() / ground_truth.area()
  
  return precision, recall

def object2index(name):
  return training_data.keys().index(name)

if __name__ == "__main__":
  import sys
  import cv2
  
  for dirpath, dirname, filenames in os.walk(sys.argv[1]):
    if 'rgb.png' in filenames:
      loadImage(dirpath)
  
  print "Training data per object:"
  for name, data in training_data.items():
    print " - %s: %d boxes" % (name, len(data))
    
  precision = {}
  recall = {}
  counts = {}

  for dirpath, dirname, filenames in os.walk(sys.argv[2]):
    if 'rgb.png' in filenames:
      frame_precision, frame_recall = testImage(dirpath)
      
      print
      print frame_precision
      print frame_recall
      
      for name, value in frame_precision.items():
	precision[name] = precision.get(name, 0.0) + value
	counts[name] = counts.get(name, 0) + 1
      for name, value in frame_recall.items():
	recall[name] = recall.get(name, 0.0) + value

  precision = { name : value / counts[name] for name, value in precision.items() }
  recall = { name : value / counts[name] for name, value in recall.items() }

  print
  print
  
  print "Precision/recall per object:"
  namelist = sorted(precision.keys())
  for name in namelist:
    print " - %35s: recall: %10.6f, precision: %10.6f" % (name, precision[name], recall[name])

  mean_prec = sum(precision.values())/len(precision)
  mean_recall = sum(recall.values())/len(recall)
  print "Mean precision:", mean_prec
  print "Mean recall:", mean_recall
  print "F-Score:", 2*(mean_prec*mean_recall)/(mean_prec+mean_recall)
