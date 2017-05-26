# kate: replace-tabs off; indent-width 4;

import json
import collections

from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg

import husl

import sys

app = QtGui.QApplication([])

win = pg.GraphicsWindow(title="Basic plotting examples")
win.resize(1920/2-20,1080/2-40)
win.setWindowTitle(sys.argv[1])

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

p1 = win.addPlot(title="Training")
p1.addLegend(offset=(-30,-30))
p1.enableAutoRange('xy', False)
p1.setXRange(0.0, 100000.0, padding=0)
p1.setYRange(0.4, 1.0, padding=0)
p1.setLabels(bottom='Iterations')
p1.showGrid(x=True, y=True)

inputFiles = sys.argv[1:]

def rgb(r,g,b):
	return r*255.0, g*255.0, b*255.0

# Setup plots
plots = {}
for idx, file in enumerate(inputFiles):
	hue = idx * 360.0 / len(inputFiles)
	color_map = husl.husl_to_rgb(hue, 80.0, 40.0)
	color_f1 = husl.husl_to_rgb(hue, 40.0, 60.0)

	text_map = pg.TextItem(anchor=(0,0.5))
	text_f1 = pg.TextItem(anchor=(0,0.5))

	p1.addItem(text_map)
	p1.addItem(text_f1)

	plots[file] = {
		'test_f1': p1.plot(pen=rgb(*color_f1), name='%s F1' % file),
		'test_map': p1.plot(pen=rgb(*color_map), name='%s mAP' % file),
		'test_f1_text': text_f1,
		'test_f1_color': rgb(*color_f1),
		'test_map_text': text_map,
		'test_map_color': rgb(*color_map),
	}

firstUpdate = True

def update():
	global plots
	global firstUpdate

	for file in inputFiles:
		j = json.JSONDecoder(object_pairs_hook=collections.OrderedDict).decode(open(file).read())

		p = plots[file]

		if 'test_results_history' in j:
			test_result_data = [ (int(float(k)), float(v['ap_results']['map']), v, float(v['f1_results']['mean_f1'])) for k,v in j['test_results_history'].items() ]
			test_result_data.sort(key=lambda x: x[0])

			p['test_map'].setData( [ x[0] for x in test_result_data ], [ x[1] for x in test_result_data ] )
			p['test_f1'].setData( [ x[0] for x in test_result_data ], [ x[3] for x in test_result_data ] )

			# Final mAP
			final_map = test_result_data[-1][1]
			p['test_map_text'].setText("%.4f" % final_map, color=p['test_map_color'])
			p['test_map_text'].setPos(test_result_data[-1][0], test_result_data[-1][1])

			print('Final mAP: %6.3f (%s)' % (final_map, file))

			# Final F1
			final_f1 = test_result_data[-1][3]
			p['test_f1_text'].setText("%.4f" % final_f1, color=p['test_f1_color'])
			p['test_f1_text'].setPos(test_result_data[-1][0], test_result_data[-1][3])

	if not firstUpdate:
		p1.enableAutoRange('xy', False)
	firstUpdate = False

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
	timer = QtCore.QTimer()
	timer.setInterval(1000)
	timer.setSingleShot(True)
	timer.timeout.connect(update)

	watcher = QtCore.QFileSystemWatcher()
	for file in inputFiles:
		watcher.addPath(file)
	watcher.fileChanged.connect(timer.start)
	update()

	import sys
	if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
		QtGui.QApplication.instance().exec_()
