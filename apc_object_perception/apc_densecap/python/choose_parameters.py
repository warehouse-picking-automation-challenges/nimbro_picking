
from hyperopt import hp, fmin, tpe, Trials
from hyperopt.mongoexp import MongoTrials

#import matplotlib.pyplot as plt
import numpy as np

trials = MongoTrials('mongo://localhost:1234/apc_densecap_db/jobs', exp_key='exp3')

space = {
  'min-iou': hp.uniform('min-iou', 0, 1),
  'C': hp.lognormal('C', 0, 1),
  'num-top-rectangles': hp.quniform('num-top-rectangles', 1, 15, 1),
  'tailor': hp.choice('tailor', [0, 1]),
  'bias-scale': hp.uniform('bias-scale', 0.0, 10.0),
}

class Experiment:
  def __init__(self, train, dataset):
    self.train = train
    self.dataset = dataset
  
  def evaluate(self, params):
    import subprocess
    import re

    proc = subprocess.run(
      [self.train,
        '--min-iou', str(params['min-iou']),
        '--C', str(params['min-iou']),
        '--num-top-rectangles', str(params['num-top-rectangles']),
        '--tailor', str(int(params['tailor'])),
        '--bias-scale', str(params['bias-scale']),
        '--dataset', self.dataset],
      stdout=subprocess.PIPE, universal_newlines=True, check=True
    )

    fScore = None
    exp = re.compile(r'F-Score: (\d+\.\d+)')
    for line in proc.stdout:
      if 'F-Score' in line:
        m = re.match(exp, line)
        fScore = float(m.group(1))

    if fScore is None:
      return 0

    return -fScore

if __name__ == "__main__":
  import sys
  
  exp = Experiment(sys.argv[1], sys.argv[2])
  
  best = fmin(
    fn=exp.evaluate,
    space=space,
    algo=tpe.suggest,
    max_evals=1000,
    trials=trials,
  )
  
  print(best)

  #parameters = ['min_iou', 'svm_c', 'num_top_rectangles']
  #f, axes = plt.subplots(nrows=2, ncols=3, figsize=(15,10))
  #cmap = plt.cm.jet
  #for i, val in enumerate(parameters):
    #xs = np.array([t['misc']['vals'][val] for t in trials.trials]).ravel()
    #ys = [-t['result']['loss'] for t in trials.trials]
    #xs, ys = zip(*sorted(zip(xs, ys)))
    #ys = np.array(ys)
    #axes[i/3,i%3].scatter(xs, ys, s=20, linewidth=0.01, alpha=0.5, c=cmap(float(i)/len(parameters)))
    #axes[i/3,i%3].set_title(val)
    ##axes[i/3,i%3].set_ylim([0.9,1.0])

  #plt.show()
