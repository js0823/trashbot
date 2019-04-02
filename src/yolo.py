# https://itnext.io/implementing-yolo-v3-in-tensorflow-tf-slim-c3c55ff59dbe

import tensorflow as tf

slim = tf.contrib.slim

# Configurations #
data_format = 'NCHW' # NHWC for gpu
BATCH_NORM_DECAY = 0.9
BATCH_NORM_EPSILON = 1e-05
LEAKY_RELU = 0.1

def darknet53(inputs):
    pass

def yolo_v3(inputs, num_classes, is_training=False, data_format=data_format, reuse=False):
    pass

def load_weights(var_list, weights_file):
    pass

