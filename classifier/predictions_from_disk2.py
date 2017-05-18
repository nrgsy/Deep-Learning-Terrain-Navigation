from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import numpy as np
import math as math
import argparse
from random import randint
import time
import tensorflow as tf
import os
import glob

from shared_functions import model
from shared_functions import get_raw_results
from shared_functions import get_results
from shared_functions import get_special_results


def file_len(fname):
    with open(fname) as f:
        for i, l in enumerate(f):
            pass
    return i + 1


def read_from_csv(filename_queue):
    reader = tf.TextLineReader(skip_header_lines=0)
    _, csv_row = reader.read(filename_queue)

    record_defaults = [[0.0]] * 885
    dict_in = tf.decode_csv(csv_row, record_defaults)
    features = dict_in[:884]
    label = dict_in[884:]
    features = tf.pack(features)
    label = tf.pack(label)
    return features, label


def input_pipeline(batch_size, dataset, num_epochs=None):
    filename_queue = tf.train.string_input_producer(
        [dataset], num_epochs=num_epochs, shuffle=True)
    example, label = read_from_csv(filename_queue)
    min_after_dequeue = 10000
    capacity = min_after_dequeue + 3 * batch_size
    example_batch, label_batch = tf.train.shuffle_batch(
        [example, label],
        batch_size=batch_size,
        capacity=capacity,
        min_after_dequeue=min_after_dequeue,
        allow_smaller_final_batch=True,
        num_threads=6)  # my machine has 6 cores
    return example_batch, label_batch

##########################################################################
# MAIN PROGRAM START
##########################################################################


program_start = time.time()

# dataset = 'dataset_closeups.csv'
dataset = 'dataset_closeups2and3.csv'

num_epochs = 1
dataset_size = file_len(dataset)
print("dataset_size  %g" % dataset_size)
examples, labels = input_pipeline(dataset_size, dataset, num_epochs)

with tf.Session() as sess:

    device_name = "/gpu:0"
    init_op = tf.group(tf.initialize_all_variables(),
                       tf.initialize_local_variables())
    init_op.run()
    coord = tf.train.Coordinator()
    threads = tf.train.start_queue_runners(coord=coord)

    # specify that CPU should be used for initial imaging
    # loading, its literally 20 times faster
    with tf.device('/cpu:0'):
        s = time.time()
        data, labels = sess.run([examples, labels])

    print('data loading and partitioning took: ' + str(time.time() - s))

    num_in_fc2 = 1024  # no second layer right now
    num_in_fc1 = 1024  # 3024 seems just as good
    with tf.name_scope('input'):
        x = tf.placeholder(tf.float32, [None, 884])
        keep_prob = tf.placeholder(tf.float32)
    with tf.name_scope('out'):
        y_conv = model(x, keep_prob, num_in_fc1, num_in_fc2)
    restorer = tf.train.Saver(tf.all_variables())

    print('initializing vars...')
    t0 = time.time()
    sess.run(tf.initialize_all_variables())
    t1 = time.time()
    print('initialize_all_variables took ----> ' + str(t1 - t0))

    # init weights with saved ones
    t0 = time.time()
    restorer.restore(
        sess,
        'saves/trained-20609_185578_35_1e-06_False_2conv1024-0.5_8207-8045-1207-1098_0.875788112303-batchNo--4095999')
    # restorer.restore(
    #     sess,
    #     'saves/trained-57340_9292_35_1e-06_False_2conv1024-0.5_392-435-43-59_0.89020452099-batchNo--1023999')

    t1 = time.time()
    print('restorer took ----> ' + str(t1 - t0))

    print('Predicting...')
    t0 = time.time()
    predictions = y_conv.eval(feed_dict={x: data, keep_prob: 1.0})
    t1 = time.time()
    print('predictions took ----> ' + str(t1 - t0))
    print(predictions)
    print(labels)

    TP, TN, FP, FN, accuracy, precision, recallSensitivity, specificity = get_results(
        predictions, labels)
    print("accuracy -------------------------------------> %g" % accuracy)
    print("precision = (TP)/(TP+FP) = %g" % precision)
    print("recallSensitivity = TP/(TP+FN) = %g" % recallSensitivity)
    print("specificity = TN/(TN+FP) = %g" % specificity)
    print("TP=" + str(TP) + ', ' + "TN=" + str(TN) +
          ', ' + "FP=" + str(FP) + ', ' + "FN=" + str(FN))

    TP, TN, FP, FN, accuracy, precision, recallSensitivity, specificity, num_uncertain = get_special_results(
        predictions, labels)
    print("special_accuracy -------------------------------------> %g" % accuracy)
    print("special_precision = (TP)/(TP+FP) = %g" % precision)
    print("special_recallSensitivity = TP/(TP+FN) = %g" % recallSensitivity)
    print("special_specificity = TN/(TN+FP) = %g" % specificity)
    print("TP=" + str(TP) + ', ' + "TN=" + str(TN) +
          ', ' + "FP=" + str(FP) + ', ' + "FN=" + str(FN))
    print("num_uncertain = %g" % num_uncertain)


end = time.time()
runtime = end - program_start
print('total program run time: ' + str(runtime))
