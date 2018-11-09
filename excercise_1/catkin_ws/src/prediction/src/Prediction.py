#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import tensorflow as tf
from keras import backend as k
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32


class Prediction:
    def __init__(self):
        self.cv_bridge = CvBridge()

        self.sensor_msgs.msg = CompressedImage()

        # SUBSCRIBE
        # subscribe to receive image data from /camera/output/specific/compressed_img_msgs
        rospy.Subscriber('camera/output/specific/compressed_img_msgs',
                         CompressedImage,
                         self.handle_received_img)

        #Subscribe to /camera/output/specific/check to verify own prediction
        rospy.Subscriber('/camera/output/specific/check',
                         Bool,
                         self.verify_own_prediction)

        # PUBLISH to '/camera/input/specific/number'
        self.publish_predicted = rospy.Publisher('/camera/input/specific/number',
                                                 Int32,
                                                 queue_size=1)

        # load keras model, filepath = "models/weights-best.hdf5"
        self.model = tf.keras.models.load_model(
            "models/weights-best.hdf5",
            custom_objects=None,
            compile=True
        )

        # manage TensorFlow threads
        self.session = k.get_session()
        self.graph = tf.get_default_graph()
        self.graph.finalize()


    def handle_received_img(self, img):
        # convert img
        image = self.cv_bridge.compressed_imgmsg_to_cv2(img)

        # use keras model to predict the content (number 0-9) on the image
        prediction = self.model.predict(image)
        # revert from one-hot encoding
        prediction_as_real_number = np.argmax(prediction, axis=None, out=None)

        # publish result of prediction to /camera/input/specific/number
        self.publish_predicted.publish(prediction_as_real_number)


    def verify_own_prediction(self, bool):
        print('prediction was: ', bool)

def main():
    try:
        # register node
        rospy.init_node('prediction', anonymous=False)

        # init Prediction
        pred = Prediction()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

        #  maybe needed??? while loop?  while not rospy.is_shutdown():

        # TODO publish/subscribe zeug, (gegenstueck zu komprimierung nutzen um das bild lesen zu koennen)

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
