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
data_loaded = False
# dataset = 'dataset_full.csv'
# dataset = 'dataset_small.csv'
# dataset = 'dataset_super_small.csv'
# dataset = 'dataset_super_small_balanced.csv'
# dataset = 'dataset_balanced.csv'
# dataset = 'dataset_balanced_merged.csv'
# dataset = 'dataset_merged012.csv'
# dataset = 'dataset_merged_0123456.csv'

# dataset = 'dataset_merged_01234567.csv'
dataset = 'dataset_closeups.csv'


output_dir = '/home/thomas/Desktop/thesis_partial/classifier/'
# output_dir = './'


base_number_batches = 2000
test_fraction = 0.1
num_epochs = 1
learning_rate = 0.00001  # 0.000001 is ok too, 0.0001 is to large sometimes
useGradientDescent = False  # AdamOptimizer seems to be better
num_in_fc2 = 1024  # no second layer right now
should_delete_checkpoints = False

# NOTE e-6 requires num_examples_to_train_on = at least 18,000,000
# e-5 requires num_examples_to_train_on = at least 5,000,000
# 800 000 000  # 80 000 000 #350 #20000000 #40000
num_examples_to_train_on = 210000000

# best params?
learning_rate = 0.000001  # 0.00001 equivalent in many cases
keep_pr = 0.5  # 0.50 #keep_pr of 0.75 seems just as good, not a big difference in tests, 0.25 good too
batch_size = 35  # 50 #seems slightly better than 20, but could be within margin of error
num_in_fc1 = 1024  # 3024 seems just as good

# TODO comment this in/out in bashrc to switch between gazeeb and tensoflow??
# export LD_LIBRARY_PATH="/usr/local/cuda-7.5/lib64"


# TODO try different kernel sizes (bigger than 5x5? the features we're looking for are more
# general) and different pooling strategies

# try adding an extra 4096 fc layer followed by the regular 1024 fc layer?

# loop to try a bunch of different configurations of parameters saving each net,
# this way i can leave and come back to a bunch of different test results
num = 0
while num <= 0:

    # if num == 0:
    #   learning_rate = 0.00001
    #   num_examples_to_train_on = 5000000 #40000
    #   keep_pr = 0.25
    # elif num == 1:
    #   learning_rate = 0.00001
    #   num_examples_to_train_on = 5000000 #40000
    #   keep_pr = 0.5
    # elif num == 2:
    #   learning_rate = 0.00001
    #   num_examples_to_train_on = 5000000 #40000
    #   keep_pr = 0.75
    # else:
    #   learning_rate = 0.000001
    #   num_examples_to_train_on = 80000000 #40000
    #   keep_pr = 0.5

    #
    # if num%9 < 3:
    #   batch_size = 20
    # elif num%9 < 6:
    #   batch_size = 35
    # else:
    #   batch_size = 50
    #
    # if num%27 < 9:
    #   keep_pr = 0.50
    # elif num%27 < 18:
    #   keep_pr = 0.25
    # else:
    #   keep_pr = 0.75
    #
    # if num%108 < 27:
    #   num_in_fc1 = 3024
    # elif num%108 < 54:
    #   num_in_fc1 = 1008
    # elif num%108 < 81:
    #   num_in_fc1 = 2016
    # else:
    #   num_in_fc1 = 4032
    #
    # if num == 109:
    #   num_examples_to_train_on = 1000000000
    #   batch_size = 50
    #   keep_pr = 0.75
    #   num_in_fc1 = 3024
    #   learning_rate = 0.00001

    num = num + 1
    # the number of batches to run
    num_iters = int(np.floor(num_examples_to_train_on / batch_size))
    dataset_size = file_len(dataset)
    print("dataset_size  %g" % dataset_size)

    # examples, labels = input_pipeline(file_length, dataset, 1)
    # examples, labels = input_pipeline(batch_size, dataset, num_epochs)
    examples, labels = input_pipeline(dataset_size, dataset, num_epochs)

    with tf.Session() as sess:

        device_name = "/gpu:0"
        # device_name = "/cpu:0"

        # To make GPU invisible: export CUDA_VISIBLE_DEVICES=""
        # To return to normal unset CUDA_VISIBLE_DEVICES
        if True:

            with tf.name_scope('input'):
                x = tf.placeholder(tf.float32, [None, 884])
                y_ = tf.placeholder(tf.float32, [None, 1])
                keep_prob = tf.placeholder(tf.float32)

            with tf.name_scope('out'):
                y_conv = model(x, keep_prob, num_in_fc1, num_in_fc2)

            with tf.name_scope("loss"):
                # could try this with sigmoid above instead of loss
                # cross_entropy = -tf.reduce_sum(y_*tf.log(y_conv))
                loss = tf.reduce_sum(tf.square(y_ - y_conv))
                # why not try some different optimizers
                if useGradientDescent:
                    optimizer = tf.train.GradientDescentOptimizer(
                        learning_rate).minimize(loss)
                else:
                    optimizer = tf.train.AdamOptimizer(
                        learning_rate).minimize(loss)
                tf.scalar_summary("loss", loss)

            # log writer. run: tensorboard
            # --logdir=/media/thomas/MasterDrive/classifier/logs/gpu_log
            writer = tf.train.SummaryWriter(
                output_dir + 'logs/gpu_log', sess.graph)
            merged = tf.merge_all_summaries()

            print(
                "***********************************************************************************1")
            # sess.run(tf.initialize_local_variables())
            init_op = tf.group(tf.initialize_all_variables(),
                               tf.initialize_local_variables())
            init_op.run()
            # tf.initialize_all_variables().run()
            print(
                "***********************************************************************************2")

            # start populating filename queue
            coord = tf.train.Coordinator()
            threads = tf.train.start_queue_runners(coord=coord)

            n_test_images = int(np.floor(test_fraction * dataset_size))
            print("n_test_images = %g" % n_test_images)
            saver = tf.train.Saver()

            i = 1
            try:
                if not data_loaded:
                    print(
                        "***********************************************************************************3")
                    # specify that CPU should be used for initial imaging
                    # loading, its literally 20 times faster
                    with tf.device('/cpu:0'):
                        s = time.time()
                        data, labels = sess.run([examples, labels])
                    print(
                        "***********************************************************************************4")

                    test_images, training_images = np.split(
                        data, [n_test_images])
                    test_labels, training_labels = np.split(
                        labels, [n_test_images])

                    training_batches_images = np.array_split(training_images,
                                                             int(np.floor(len(training_images) / batch_size)))
                    training_batches_labels = np.array_split(training_labels,
                                                             int(np.floor(len(training_labels) / batch_size)))
                    data_loaded = True

                    print('data loading and partitioning took: ' +
                          str(time.time() - s))
                    print(
                        "***********************************************************************************5")
                else:
                    print(
                        'skipping data loading step, tis already loaded from previous iteration')

                if (len(training_batches_labels) != len(training_batches_images)):
                    print(
                        'ERROR BATCH SPLIT FAILED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

                filestring = '_' + str(dataset_size) + '_' + str(batch_size) + \
                    '_' + str(learning_rate) + '_' + str(useGradientDescent) + '_' + '2conv' + str(num_in_fc1) + \
                    '-' + str(keep_pr) + '_'

                print('filestring: ' + filestring)

                start = time.time()

                j = 0
                for i in range(num_iters):

                    # pick random location to get the batch
                    index = randint(0, len(training_batches_images) - 1)
                    image_batch = training_batches_images[index]
                    label_batch = training_batches_labels[index]
                    if (len(image_batch) != len(label_batch)):
                        print(
                            'ERROR BATCH SPLIT FAILED2!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

                    # s = time.time()
                    optimizer.run(
                        feed_dict={x: image_batch, y_: label_batch, keep_prob: keep_pr})
                    # print('t: ' + str(time.time() - s))

                    if i % 1000 == 0:
                        pred = y_conv.eval(
                            feed_dict={x: image_batch, keep_prob: 1.0})
                        TP, TN, FP, FN = get_raw_results(pred, label_batch)
                        train_accuracy = (TP + TN) / (TP + TN + FP + FN)
                        print("%d of %d batches complete, train acc: %g" %
                              (i, num_iters, train_accuracy))

                    # increase the time between saves exponentially, this way we never are saving too many
                    # checkpoints regardless of how many iterations were
                    # running
                    exportNum = base_number_batches * math.pow(2, j)
                    if i % exportNum == exportNum - 1:
                        end = time.time()
                        runtime = end - start
                        pred = y_conv.eval(
                            feed_dict={x: test_images, keep_prob: 1.0})
                        TP, TN, FP, FN, accuracy, precision, recallSensitivity, specificity = \
                            get_results(pred, test_labels)
                        print('exporting graph... run time: ' + str(runtime) + " TP=" + str(TP) + ', ' +
                              "TN=" + str(TN) + ', ' + "FP=" + str(FP) + ', ' + "FN=" + str(FN) + ', accuracy=' +
                              str(int(100 * accuracy)))

                        saver.save(sess, output_dir + 'saves/trained-' + str(int(runtime)) + filestring +
                                   str(TP) + '-' + str(TN) + '-' + str(FP) + '-' + str(FN) + '_' +
                                   str(accuracy) + '-batchNo-', global_step=i)
                        j = j + 1

                    i = i + 1
            except tf.errors.OutOfRangeError:
                print('Done training, epoch limit reached')
            finally:
                coord.request_stop()
            coord.join(threads)

            end = time.time()
            runtime = end - start
            print('round run time: ' + str(runtime))
            pred = y_conv.eval(feed_dict={x: test_images, keep_prob: 1.0})
            TP, TN, FP, FN, accuracy, precision, recallSensitivity, specificity = get_results(
                pred, test_labels)
            print(
                "final test accuracy --------------------------------------------%g" % accuracy)
            print("final test precision = (TP)/(TP+FP) %g" % precision)
            print("final test recallSensitivity = TP/(TP+FN) %g" %
                  recallSensitivity)
            print("final specificity = TN/(TN+FP) %g" % specificity)
            print("TP=" + str(TP) + ', ' + "TN=" + str(TN) +
                  ', ' + "FP=" + str(FP) + ', ' + "FN=" + str(FN))

            # Saves the final checkpoint, which by default also exports a
            # meta_graph
            saver.save(sess, output_dir + 'saves/FINAL-' + str(int(runtime)) + filestring + str(TP) +
                       '-' + str(TN) + '-' + str(FP) + '-' + str(FN) + '_' + str(accuracy), global_step=i)

            # delete checkpoints if we make it here? they're really only useful
            # if something crashes
            if should_delete_checkpoints:
                filelist = glob.glob(output_dir + 'saves/trained*')
                for f in filelist:
                    os.remove(f)
                print('DONE WITH ROUND')

end = time.time()
runtime = end - program_start
print('total program run time: ' + str(runtime))
