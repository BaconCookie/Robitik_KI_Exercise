#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import tensorflow as tf
from keras import backend as k
from keras.models import load_model
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32

filepath = '/home/laura/PycharmProjects/Robotik_KI_Exercise/excercise_1/ai_train/models/weights-best.hdf5'
predicted_randoms = []
PUBLISH_RATE = 3  # hz


class Prediction:

    def __init__(self):
        self.cv_bridge = CvBridge()

        ### SPECIFIC IMAGE ##############################################################
        # SUBSCRIBE to receive image data from /camera/output/specific/compressed_img_msgs
        rospy.Subscriber('camera/output/specific/compressed_img_msgs',
                         CompressedImage,
                         self.handle_received_img_specific)

        # SUBSCRIBE to /camera/output/specific/check to verify own prediction
        rospy.Subscriber('/camera/output/specific/check',
                         Bool,
                         self.verify_own_prediction_specific)

        # PUBLISH to '/camera/input/specific/number'
        self.publish_predicted_specific = rospy.Publisher('/camera/input/specific/number',
                                                 Int32,
                                                 queue_size=1)
        #################################################################################

        ### RANDOM IMAGE ################################################################
        # SUBSCRIBE to /camera/output/random/number to verify own prediction
        rospy.Subscriber('/camera/output/random/number',
                         Int32,
                         self.verify_own_prediction_random)

        # SUBSCRIBE to receive image data from /camera/output/random/compressed_img_msgs
        rospy.Subscriber('/camera/output/random/compressed_img_msgs',
                         CompressedImage,
                         self.handle_received_img_random)
        #################################################################################


        # load keras model, filepath = "models/weights-best.hdf5"
        self.model = load_model(filepath = filepath)

        # manage Keras/TensorFlow threads
        # https://stackoverflow.com/questions/46725323/keras-tensorflow-exception-while-predicting-from-multiple-threads
        self.model._make_predict_function()
        self.session = k.get_session()
        self.graph = tf.get_default_graph()
        self.graph.finalize()

    def convert_image(self, img):
        a = self.cv_bridge.compressed_imgmsg_to_cv2(img) #('a', (28, 28))
        b = np.expand_dims(a, axis=2) #('b', (28, 28, 1))
        c = np.expand_dims(b, axis=0) #('c', (1, 28, 28, 1))
        return c

    def handle_received_img_specific(self, img):
        # convert img
        image = self.convert_image(img)
        # use keras model to predict the content (number 0-9) on the image
        prediction = self.model.predict(image)
        # revert from one-hot encoding
        prediction_as_real_number = np.argmax(prediction, axis=None, out=None)
        # publish result of prediction to /camera/input/specific/number
        self.publish_predicted_specific.publish(prediction_as_real_number)

    def handle_received_img_random(self, img):
        # convert img
        image = self.convert_image(img)
        # use keras model to predict the content (number 0-9) on the image
        prediction = self.model.predict(image)
        # revert from one-hot encoding
        prediction_as_real_number = np.argmax(prediction, axis=None, out=None)
        # for verification: save value in global scope of this class
        global predicted_randoms
        predicted_randoms.append(prediction_as_real_number)


    def verify_own_prediction_specific(self, bool):
        print'Prediction of specific number was: {}'.format(bool)

    def verify_own_prediction_random(self, number):
        print('######  Prediction of random number:  ######')
        # add sleep to the verification of random numbers for synchronization with the camera
        rate = rospy.Rate(PUBLISH_RATE)
        rate.sleep()
        try:
            self.verify_helper(number, -2)

        except (IndexError):
            try:
                self.verify_helper(number, -1)
                # add a little bit of extra sleep during startup phase
                extra_sleep_rate = rospy.Rate(10) # hz
                extra_sleep_rate.sleep()

            except (IndexError):
                print('Patience please!')
            pass

    def verify_helper(self, number, index):
        number = number.data
        predicted = predicted_randoms.__getitem__(index)
        result = True if number == predicted else False
        print'Saved predicted randoms:{}'.format(predicted_randoms)
        print'Actual: {} Predicted: {}'.format(number, predicted)
        print'Prediction of random number was: {}'.format(result)
        print('############################################')

def main():
    try:
        # register node
        rospy.init_node('prediction', anonymous=False)

        # init Prediction
        pred = Prediction()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
