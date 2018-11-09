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

        # load keras model, filepath = "models/weights-best.hdf5"
        model = tf.keras.models.load_model(
            "models/weights-best.hdf5",
            custom_objects=None,
            compile=True
        )


def main():
    try:
        # register node
        rospy.init_node('prediction', anonymous=False)

        # init CameraPseudo
        pred = Prediction()

        rospy.spin()

        # TODO while loop
        while not rospy.is_shutdown():
            print('rospy lives, while loop in Prediction')

            # TODO publish/subscribe zeug, (gegenstück zu komprimierung nutzen um das bild lesen zu können)

            # TODO benutze keras modell zum predicten

            # TODO threads managen! sehe link in readme

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
