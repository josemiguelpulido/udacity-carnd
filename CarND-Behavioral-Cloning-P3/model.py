import csv
import cv2
import numpy as np
import sys


data_dir = sys.argv[1]
model_name = sys.argv[2]

print('Reading images and steering angle measurements...')

def get_image(image_name):
    image_path = data_dir + '/IMG/' + image_name
    return cv2.imread(image_path)

def generator(samples, batch_size=32):
    
    num_samples = len(samples)
    shuffle(samples)

    while 1: # loop forever so the generator never terminates

        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset : offset + batch_size]

            images = []
            measurements = []

            augmented_images, augmented_measurements = [], []

            for sample in batch_samples:

                # get the corresponding steering angle measurement
                measurement = float(sample[3])

                # ignore samples with 0 steering angle to force the network to always try to adjust
                #if measurement == 0:
                #    continue
                
                correction = 0.125
                measurement_left = measurement + correction
                measurement_right = measurement - correction
                #measurements.append(measurement)
                measurements.extend([measurement, measurement_left, measurement_right])

                # get the center image
                image_name = sample[0].split('/')[-1]
                image = get_image(image_name)

                # get the left and right images
                image_name = sample[1].split('/')[-1]
                image_left = get_image(image_name)

                image_name = sample[2].split('/')[-1]
                image_right = get_image(image_name)

                #images.append(image)
                images.extend([image, image_left, image_right])


            # add flipped image and corresponding corrected steeting angle to augment dataset,                
            # and to prevent bias to steer to the left
            for img, meas in zip(images, measurements):
                augmented_images.append(img)
                augmented_measurements.append(meas)
                augmented_images.append(cv2.flip(img,1))
                augmented_measurements.append(meas * -1.0)
                
            # convert data into numpy arrays required by Keras
            X = np.array(augmented_images)
            y = np.array(augmented_measurements)
            yield (X, y)

samples = list(csv.reader(open(data_dir + '/driving_log.csv')))

from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

train_samples, validation_samples = train_test_split(samples, test_size=0.2)

print('num training samples: {}'.format(len(train_samples)))
print('num validation samples: {}'.format(len(validation_samples)))

train_generator_data = generator(train_samples)
validation_generator_data = generator(validation_samples)

from keras.models import Sequential
from keras.layers import Dense, Flatten, Lambda, Dropout, Cropping2D
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D

# lenet model
model = Sequential()

# normalize data (0 centered, small deviation)
model.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=(160,320,3)))

# cropping 50 pixels from top of image, 20px from bottom, 0 from left or right
model.add(Cropping2D(cropping=((40,10), (0,0))))

# network
model.add(Convolution2D(6,5,5, activation='relu'))
#model.add(Dropout(0.3))
model.add(MaxPooling2D())
model.add(Convolution2D(6,5,5, activation='relu'))
#model.add(Dropout(0.3))
model.add(MaxPooling2D())
model.add(Flatten())
model.add(Dense(120))
model.add(Dense(84))
model.add(Dense(1))


# nvidia model
model2 = Sequential()

model2.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=(160,320,3)))
model2.add(Cropping2D(cropping=((70,25), (0,0))))
model2.add(Convolution2D(24,5,5, subsample=(2,2),activation='relu'))
model2.add(Convolution2D(36,5,5, subsample=(2,2),activation='relu'))
model2.add(Convolution2D(48,5,5, subsample=(2,2),activation='relu'))
model2.add(Dropout(0.3))
model2.add(Convolution2D(64,3,3, activation='relu'))
model2.add(Convolution2D(64,3,3, activation='relu'))
model2.add(Dropout(0.3))
model2.add(Flatten())
model2.add(Dense(100))
model2.add(Dense(50))
model2.add(Dense(10))
model2.add(Dense(1))
    
# use MSE since this model outputs a single continuous numeric value
model2.compile(loss='mse', optimizer='adam')

model2.fit_generator(train_generator_data, samples_per_epoch= len(6*train_samples),
                    validation_data=validation_generator_data, nb_val_samples=len(6*validation_samples), nb_epoch=6, verbose=1)
model2.save(model_name)

