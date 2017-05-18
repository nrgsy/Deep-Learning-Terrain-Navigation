import tensorflow as tf


def model(data, keep_prob, num_in_fc1, num_in_fc2):

    # 5x5 convolutions
    with tf.name_scope("conv1"):
        W_conv1 = weight_variable([5, 5, 1, 32])
        b_conv1 = bias_variable([32])
        x_image = tf.reshape(data, [-1, 26, 34, 1])
        tf.image_summary('input', x_image, 20)

        h_conv1 = tf.nn.relu(conv2d(x_image, W_conv1) + b_conv1)
        tf.histogram_summary("W_conv1_summ", W_conv1)
        tf.histogram_summary("b_conv1_summ", b_conv1)
    with tf.name_scope("pool1"):
        h_pool1 = max_pool_2x2(h_conv1)
    with tf.name_scope("conv2"):
        W_conv2 = weight_variable([5, 5, 32, 64])
        b_conv2 = bias_variable([64])
        h_conv2 = tf.nn.relu(conv2d(h_pool1, W_conv2) + b_conv2)
        tf.histogram_summary("W_conv2_summ", W_conv2)
        tf.histogram_summary("b_conv2_summ", b_conv2)
    with tf.name_scope("pool2"):
        h_pool2 = max_pool_2x2(h_conv2)
    # with tf.name_scope("conv3"):
    #   W_conv3 = weight_variable([5, 5, 64, 128])
    #   b_conv3 = bias_variable([128])
    #   h_conv3 = tf.nn.relu(conv2d(h_pool2, W_conv3) + b_conv3)
    #   tf.histogram_summary("W_conv3_summ", W_conv3)
    #   tf.histogram_summary("b_conv3_summ", b_conv3)

    # 5*5*64 = 1600 = 4*400 = 4*20*20 just like #7*7*64 = 3136 = 4*784 = 4*28*28
    # start with 400-pixel image (20x20) -conv1>
    # 32*20*20 pixel images -pool1> 32*10*10 images -conv2> 64*10*10 images
    # -pool2> 64*5*5 = 1600

    # 3536 = 4*884 = 4*26*34
    # start with 884-pixel image (26x34) -conv1>
    # 32*26*34 pixel images -pool1> 32*13*17 images -conv2> 64*13*17 images -pool2> 64*7*9 = 4032 ->
    # 128x7x9 = 8064

# uncomment for 1 fully connected layer at the end
    with tf.name_scope("fc1"):
        W_fc1 = weight_variable([4032, num_in_fc1])
        b_fc1 = bias_variable([num_in_fc1])
        h_pool2_flat = tf.reshape(h_pool2, [-1, 4032])
        h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + b_fc1)
        h_fc1_drop = tf.nn.dropout(h_fc1, keep_prob)
        tf.histogram_summary("W_fc1_summ", W_fc1)
        tf.histogram_summary("b_fc1_summ", b_fc1)
    with tf.name_scope("fc2"):
        W_fc2 = weight_variable([num_in_fc1, 1])
        b_fc2 = bias_variable([1])
        tf.histogram_summary("W_fc2_summ", W_fc2)
        tf.histogram_summary("b_fc2_summ", b_fc2)
        # y_conv=tf.nn.softmax(tf.matmul(h_fc1_drop, W_fc2) + b_fc2)
        return tf.sigmoid(tf.matmul(h_fc1_drop, W_fc2) + b_fc2)

# uncomment for 2 fully connected layers at the end
    # with tf.name_scope("fc1"):
    #   W_fc1 = weight_variable([4032, num_in_fc1])
    #   b_fc1 = bias_variable([num_in_fc1])
    #   h_pool2_flat = tf.reshape(h_pool2, [-1, 4032])
    #   h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + b_fc1)
    #   tf.histogram_summary("W_fc1_summ", W_fc1)
    #   tf.histogram_summary("b_fc1_summ", b_fc1)
    # with tf.name_scope("fc2"):
    #   W_fc2 = weight_variable([num_in_fc1, num_in_fc2])
    #   b_fc2 = bias_variable([num_in_fc2])
    #   h_fc1_flat = tf.reshape(h_fc1, [-1, num_in_fc1])
    #   h_fc2 = tf.nn.relu(tf.matmul(h_fc1_flat, W_fc2) + b_fc2)
    #   h_fc2_drop = tf.nn.dropout(h_fc2, keep_prob)
    #   tf.histogram_summary("W_fc2_summ", W_fc2)
    #   tf.histogram_summary("b_fc2_summ", b_fc2)
    # with tf.name_scope("fc3"):
    #   W_fc3 = weight_variable([num_in_fc2, 1])
    #   b_fc3 = bias_variable([1])
    #   tf.histogram_summary("W_fc3_summ", W_fc3)
    #   tf.histogram_summary("b_fc3_summ", b_fc3)
    #   # y_conv=tf.nn.softmax(tf.matmul(h_fc1_drop, W_fc2) + b_fc2)
    #   return tf.sigmoid(tf.matmul(h_fc2_drop, W_fc3) + b_fc3)


def weight_variable(shape):
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.Variable(initial)


def bias_variable(shape):
    initial = tf.constant(0.1, shape=shape)
    return tf.Variable(initial)


def max_pool_2x2(x):
    return tf.nn.max_pool(x, ksize=[1, 2, 2, 1],
                          strides=[1, 2, 2, 1], padding='SAME')


def conv2d(x, W):
    return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')


def round_predictions(predictions):
    threshold = 0.5
    for i in range(0, len(predictions)):
        pred = predictions[i]
        if (pred > threshold):
            predictions[i][0] = 1
        else:
            predictions[i][0] = 0
    return predictions


def get_raw_results(predictions, labels):
    threshold = 0.5
    TP = 0
    TN = 0
    FP = 0
    FN = 0

    for i in range(0, len(predictions)):
        pred = predictions[i]
        label = labels[i]
        if (label[0] == 1):
            if (pred > threshold):
                TP = TP + 1
            else:
                FN = FN + 1
        elif (label[0] == 0):
            if (pred > threshold):
                FP = FP + 1
            else:
                TN = TN + 1
        else:
            print('ERROR: bad label!!!!!!!!!!!!!!!')

    return TP, TN, FP, FN


def get_results(predictions, labels):

    TP, TN, FP, FN = get_raw_results(predictions, labels)

    accuracy = ((float)(TP + TN) /
                (float)(TP + TN + FP + FN))  # what % did you get right

    if (TP + FP != 0):
        # Of the ones you labeled drivable, how many were correct?
        precision = ((float)(TP)) / ((float)(TP + FP))
    else:
        precision = -1

    if (TP + FN != 0):
        # TPR, Of the drivable ones, how many were labeled correctly?
        recallSensitivity = ((float)(TP)) / ((float)(TP + FN))
    else:
        recallSensitivity = -1

    if (TP + FP != 0):
        # TNR, Of the undrivable ones, how many were labeled correctly?
        specificity = ((float)(TN)) / ((float)(TN + FP))
    else:
        specificity = -1

    return TP, TN, FP, FN, accuracy, precision, recallSensitivity, specificity


def get_special_results(predictions, labels):
    lower_threshold = 0.1
    upper_threshold = 0.9

    TP = 0
    TN = 0
    FP = 0
    FN = 0

    num_uncertain = 0

    for i in range(0, len(predictions)):
        pred = predictions[i]
        label = labels[i]
        if (label[0] == 1):
            if (pred > upper_threshold):
                TP = TP + 1
            elif (pred < lower_threshold):
                FN = FN + 1
            else:
                num_uncertain = num_uncertain + 1
        elif (label[0] == 0):
            if (pred > upper_threshold):
                FP = FP + 1
            elif (pred < lower_threshold):
                TN = TN + 1
            else:
                num_uncertain = num_uncertain + 1
        else:
            print('ERROR: bad label!!!!!!!!!!!!!!!')

    accuracy = ((float)(TP + TN) /
                (float)(TP + TN + FP + FN))  # what % did you get right

    if (TP + FP != 0):
        # Of the ones you labeled drivable, how many were correct?
        precision = ((float)(TP)) / ((float)(TP + FP))
    else:
        precision = -1

    if (TP + FN != 0):
        # TPR, Of the drivable ones, how many were labeled correctly?
        recallSensitivity = ((float)(TP)) / ((float)(TP + FN))
    else:
        recallSensitivity = -1

    if (TP + FP != 0):
        # TNR, Of the undrivable ones, how many were labeled correctly?
        specificity = ((float)(TN)) / ((float)(TN + FP))
    else:
        specificity = -1

    return TP, TN, FP, FN, accuracy, precision, recallSensitivity, specificity, num_uncertain
