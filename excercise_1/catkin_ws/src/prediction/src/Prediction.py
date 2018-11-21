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
current_predicted_random_number = 42


class Prediction:


    def __init__(self):
        self.cv_bridge = CvBridge()

        #Todo check if following is needed:
        # self.sensor_msgs.msg = CompressedImage()


        ### SPECIFIC IMAGE ###
        # SUBSCRIBE
        # subscribe to receive image data from /camera/output/specific/compressed_img_msgs
        rospy.Subscriber('camera/output/specific/compressed_img_msgs',
                         CompressedImage,
                         self.handle_received_img_specific)

        #Subscribe to /camera/output/specific/check to verify own prediction
        rospy.Subscriber('/camera/output/specific/check',
                         Bool,
                         self.verify_own_prediction_specific)

        # PUBLISH to '/camera/input/specific/number'
        self.publish_predicted_specific = rospy.Publisher('/camera/input/specific/number',
                                                 Int32,
                                                 queue_size=1)
        ### ----- ###


        ### RANDOM IMAGE ###
        # SUBSCRIBE
        # subscribe to receive image data from /camera/output/random/compressed_img_msgs
        rospy.Subscriber('/camera/output/random/compressed_img_msgs',
                         CompressedImage,
                         self.handle_received_img_random)

        # Subscribe to /camera/output/random/number to verify own prediction
        rospy.Subscriber('/camera/output/random/number',
                         Int32,
                         self.verify_own_prediction_random)

        # # PUBLISH to '/camera/input/random/number'
        # self.publish_predicted_random = rospy.Publisher('/camera/input/random/number',
        #                                          Int32,
        #                                          queue_size=1)

        ### ----- ###


        # load keras model, filepath = "models/weights-best.hdf5"
        self.model = load_model(filepath = filepath)
        # self.model = tf.keras.models.load_model(
        #     "weights-best.hdf5",
        #     custom_objects=None,
        #     compile=True
        # )

        # manage Keras/TensorFlow threads
        # https://stackoverflow.com/questions/46725323/keras-tensorflow-exception-while-predicting-from-multiple-threads
        self.model._make_predict_function()
        self.session = k.get_session()
        self.graph = tf.get_default_graph()
        self.graph.finalize()


    def convert_image(self, img):
        a = self.cv_bridge.compressed_imgmsg_to_cv2(img)
        #print('a',a.shape)                  #('a', (28, 28))
        b = np.expand_dims(a, axis=2)
        #print('b',b.shape)                  #('b', (28, 28, 1))
        c = np.expand_dims(b, axis=0)
        #print('c',c.shape)                  #('c', (1, 28, 28, 1))
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
        global current_predicted_random_number
        current_predicted_random_number = prediction_as_real_number

        # # publish result of prediction to /camera/input/specific/number
        # self.publish_predicted_random.publish(prediction_as_real_number)


    def verify_own_prediction_specific(self, bool):
        print('Prediction of specific number was: ', bool)


    def verify_own_prediction_random(self, number):
        number = number.data
        result =  True if number == current_predicted_random_number else False
        print('Prediction of random number was: ', number, current_predicted_random_number, result)


def main():
    try:
        # register node
        rospy.init_node('prediction', anonymous=False)

        # init Prediction
        pred = Prediction()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

        #Todo check if while loop /function calls are needed
        # while not rospy.is_shutdown():
        #     pred.publish_predicted_specific


    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
