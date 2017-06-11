import pickle
import tensorflow as tf
from sklearn.model_selection import train_test_split
from alexnet import AlexNet

# TODO: Load traffic signs data.
training_file = '/Users/josemiguelpulido/data/udacity/CarND-Traffic-Sign-Classifier-Project/traffic-signs-data/train.p'

with open(training_file, mode='rb') as f:
    train = pickle.load(f)

X_train, y_train = train['features'], train['labels']

# TODO: Split data into training and validation sets.
validation_file= '/Users/josemiguelpulido/data/udacity/CarND-Traffic-Sign-Classifier-Project/traffic-signs-data/valid.p'
with open(validation_file, mode='rb') as f:
    valid = pickle.load(f)
X_valid, y_valid = valid['features'], valid['labels']

# TODO: Define placeholders and resize operation.
x = tf.placeholder(tf.float32, (None, 32, 32, 3))
resized = tf.image.resize_images(x,[227,227])

# TODO: pass placeholder as first argument to `AlexNet`.
fc7 = AlexNet(resized, feature_extract=True)
# NOTE: `tf.stop_gradient` prevents the gradient from flowing backwards
# past this point, keeping the weights before and up to `fc7` frozen.
# This also makes training faster, less work to do!
fc7 = tf.stop_gradient(fc7)

# TODO: Add the final layer for traffic sign classification.
nb_classes = 43
shape = (fc7.get_shape().as_list()[-1], nb_classes)  
fc8W = tf.Variable(tf.truncated_normal(shape=shape))
fc8b = tf.Variable(tf.zeros(nb_classes))

logits = tf.matmul(fc7, fc8W) + fc8b

# TODO: Define loss, training, accuracy operations.
# HINT: Look back at your traffic signs project solution, you may
# be able to reuse some the code.
rate = 0.001

# placeholder for labels
y = tf.placeholder(tf.int32, (None))
one_hot_y = tf.one_hot(y, 43)

cross_entropy = tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=one_hot_y)
loss_operation = tf.reduce_mean(cross_entropy)
optimizer = tf.train.AdamOptimizer(learning_rate = rate)
training_operation = optimizer.minimize(loss_operation)

# evaluate function to be used in training, validation and testing

correct_prediction = tf.equal(tf.argmax(logits, 1), tf.argmax(one_hot_y, 1))
accuracy_operation = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))
saver = tf.train.Saver()

EPOCHS = 10
BATCH_SIZE = 128

def evaluate(X_data, y_data):
    num_examples = len(X_data)
    total_accuracy = 0
    sess = tf.get_default_session()
    for offset in range(0, num_examples, BATCH_SIZE):
        batch_x, batch_y = X_data[offset:offset+BATCH_SIZE], y_data[offset:offset+BATCH_SIZE]
        accuracy = sess.run(accuracy_operation, feed_dict={x: batch_x, y: batch_y})
        total_accuracy += (accuracy * len(batch_x))
    return total_accuracy / num_examples


# TODO: Train and evaluate the feature extraction model.

from sklearn.utils import shuffle

with tf.Session() as sess:
    sess.run(tf.global_variables_initializer())
    num_examples = len(X_train)
    
    print("Training...")
    print()
    for i in range(EPOCHS):

        # stochastic gradient descent
        X_train, y_train = shuffle(X_train, y_train)

        # train on batches
        for offset in range(0, num_examples, BATCH_SIZE):
            
            # prepare batch
            end = offset + BATCH_SIZE
            batch_x, batch_y = X_train[offset:end], y_train[offset:end]

            # train on batch
            sess.run(training_operation, feed_dict={x: batch_x, y: batch_y})
        
        # use validation set to determine accuracy
        validation_accuracy = evaluate(X_valid, y_valid)
        print("EPOCH {} ...".format(i+1))
        print("Validation Accuracy = {:.3f}".format(validation_accuracy))
        print()
        
    saver.save(sess, './alexnet_trafficsigns')
    print("Model saved")

#validate

# get test data
testing_file = '/Users/josemiguelpulido/data/udacity/CarND-Traffic-Sign-Classifier-Project/traffic-signs-data/test.p'
with open(testing_file, mode='rb') as f:
    test = pickle.load(f)
X_test, y_test = test['features'], test['labels']

# evaluate with test data
with tf.Session() as sess:
    saver.restore(sess, tf.train.latest_checkpoint('.'))

    test_accuracy = evaluate(X_test, y_test)
    print("Test Accuracy = {:.3f}".format(test_accuracy))
