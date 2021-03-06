"""
Trains a simple convnet on the MNIST dataset.

Gets to 99.25% test accuracy after 12 epochs
(there is still a lot of margin for parameter tuning).
16 seconds per epoch on a GRID K520 GPU.
"""

from __future__ import print_function

import keras
import numpy as np
from keras import backend as k
from keras import callbacks
from keras.datasets import mnist
from keras.layers import Conv2D, MaxPooling2D
from keras.layers import Dense, Dropout, Flatten
from keras.models import Sequential

# params
batch_size = 128
num_classes = 10
epochs = 12
verbose_train = 1
verbose_eval = 0

# dirs and paths

file_path = "models/weights-best.hdf5"

### data transformation ###

# input image dimensions
img_rows, img_cols = 28, 28

# the data, split between train and test sets
(x_train, y_train), (x_test, y_test) = mnist.load_data()

if k.image_data_format() == 'channels_first':  #in case theano is being used
    x_train = x_train.reshape(x_train.shape[0], 1, img_rows, img_cols)
    x_test = x_test.reshape(x_test.shape[0], 1, img_rows, img_cols)
    input_shape = (1, img_rows, img_cols)
else:
    x_train = x_train.reshape(x_train.shape[0], img_rows, img_cols, 1)
    x_test = x_test.reshape(x_test.shape[0], img_rows, img_cols, 1)
    input_shape = (img_rows, img_cols, 1)

x_train = x_train.astype('float32')
x_test = x_test.astype('float32')
#normalize image for optimal hardware performance
x_train /= 255.
x_test /= 255.
print('x_train shape:', x_train.shape)
print(x_train.shape[0], 'train samples')
print(x_test.shape[0], 'test samples')

# convert class vectors to binary class matrices
y_train = keras.utils.to_categorical(y_train, num_classes)
y_test = keras.utils.to_categorical(y_test, num_classes)

### building the model ###

model = Sequential()
model.add(Conv2D(filters=32,
                 kernel_size=(3, 3),
                 activation='relu',
                 input_shape=input_shape))
model.add(Conv2D(filters=64,
                 kernel_size=(3, 3),
                 activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(rate=0.25))
model.add(Flatten())
#dense == fully connected
model.add(Dense(units=128,
                activation='relu'))
model.add(Dropout(rate=0.5))
model.add(Dense(units=num_classes,
                activation='softmax'))

model.compile(loss=keras.losses.categorical_crossentropy,
              optimizer=keras.optimizers.Adadelta(),
              metrics=['accuracy'])

### model is ready ###

# callbacks
# for debugging: tensorboard
cb_tensorboard = callbacks.TensorBoard(write_images=True)

cb_checkpoint = callbacks.ModelCheckpoint(
    filepath=file_path,
    monitor='val_acc',
    save_best_only=True,
    save_weights_only=False,
    mode='max')

callbacks = [cb_tensorboard, cb_checkpoint]

# training
model.fit(x_train,
          y_train,
          validation_data=(x_test, y_test),
          batch_size=batch_size,
          epochs=epochs,
          # callbacks=callbacks,
          verbose=verbose_train)

# evaluation
score = model.evaluate(x_test,
                       y_test,
                       verbose=verbose_eval)
print('Test loss:', score[0])
print('Test accuracy:', score[1])

# data to choose
index = 0

# expand dimension for batch
input_data = np.expand_dims(x_test[index], axis=0)  # tensorflow, add dimension batch size for prediction
input_label = y_test[index]

# example prediction call
prediction = model.predict(input_data)

# revert from one-hot encoding
prediction = np.argmax(prediction, axis=None, out=None)
input_label = np.argmax(input_label, axis=None, out=None)

# output
print("prediction: %s" % (prediction,))
print("real label: %s" % (input_label,))

