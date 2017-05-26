
apc_densecap
============

This is an object detection framework developed for the APC 2016, based on the
very nice DenseCap framework released by Justin Johnson:
https://github.com/jcjohnson/densecap

Essentially, the language generation part is removed and replaced with a
classification head, which turns the system into an object detector.

The script `preprocess_apc.py` can be used to preprocess an APC dataset.
See `preprocess_apc_splits.sh` for usage.

If needed, a depth CNN can be trained using `train_cross_modal.lua`, under
supervision from a pretrained RGB CNN.

Finally, `train_detection.lua` trains an object detection model, and
`evaluate_model.lua` can be used to perform evaluation on a test split.
See `train.sh` for usage examples.

License
-------

To stay compatible with the original DenseCap license, all code in this directory
is released under the MIT license (see LICENSE).
