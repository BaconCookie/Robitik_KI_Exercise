#Solution of Exercise 1-3

## The ROS (Robot Operating System) basics 
ROS is the Robot Operating System. "ROS is a set of software libraries and tools that help 
you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has 
what you need for your next robotics project."

ROS is open source and has a distributed, modular design, which means that you can choose which parts that benefit you and 
your project and which parts you'd rather implement yourself. The distributed nature of ROS also fosters a large 
community of user-contributed packages that add a lot of value on top of the core ROS system. 

The ROS core:
At the lowest level, ROS offers a message passing interface that provides inter-process communication and is commonly 
referred to as a middleware.

The ROS middleware provides these facilities:
   * publish/subscribe anonymous message passing
   * recording and playback of messages
   * request/response remote procedure calls
   * distributed parameter system 

Sources:
- http://www.ros.org/, last visited 11.11.2018
- http://www.ros.org/is-ros-for-me/, last visited 11.11.2018
- http://www.ros.org/core-components/, last visited 11.11.2018


## Publish/Subscribe Principle

In software architecture, publish–subscribe is a messaging pattern where senders of messages, called publishers, do not 
program the messages to be sent directly to specific receivers, called subscribers, but instead categorize published 
messages onto topics without knowledge of their subscribers. Similarly, subscribers express 
interest in one or more topics and only receive messages that are of interest, without knowledge the publishers.

Publish–subscribe is a sibling of the message queue paradigm, and is typically one part of a larger message-oriented 
middleware system.

This pattern provides greater network scalability and a more dynamic network topology, with a resulting decreased 
flexibility to modify the publisher and the structure of the published data.
Publishers have less flexibility to be modified since subscribers rely on the topic name and the data type. 
Also, the message queue size needs to be appropriate/ suitable of the given situation.

ROS uses this principle to enable communication between ROS-nodes 
(Node = ROS term for an executable that is connected to the ROS network).
In this example the ROS-nodes are the CameraPseudo and the Prediction.
I think it is efficient and gives the engineer an reasonable way to have an overview  and stay on top of the communication 
one needs for a more complex robot.

Sources: 
- http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29, last visited 11.11.2018
- https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern, last visited 11.11.2018


## Implementation
#### Supervised Learning
Training of a model with a CNN (Convolutional Neural Network) on the MNIST dataset, using the Keras API with TensorFlow as backend.

MNIST = Dataset of 60.000 handwritten digits (0-9)

#### My Publisher/Subscriber implementation in ROS
* **_CameraPseudo_** uses MNIST data as pseudo webcam images & their correct labels for verification of the above mentioned trained model.
     
    **In case of a specific number:**
    * publishes an image (as a CompressedImage) of a specific number to camera/output/specific/compressed_img_msgs
    * subscribes to /camera/input/specific/number, here CameraPseudo receives the predicted number from the Prediction class
    * verifies if the predicted numer is the same as the label of the published image from the dataset (True if correct, False otherwise)
    * publishes this boolean value to topic /camera/output/specific/check

    **In case of a random number:**
    * publishes an image (as a CompressedImage) of a random number from the MNIST dataset to /camera/output/random/compressed_img_msgs
    * publishes the corresponding label of the above mentioned image to /camera/output/random/number
    
* **_Prediction_** 

    Summary: includes the above mentioned trained model and predicts the value based on the subscribed image inside the 
    subscribers callback 
    
    **In case of a specific number:**
    * subscribes to this topic to receive image data from /camera/output/specific/compressed_img_msgs
    * converts the CompressedImage to cv2 format
    * predicts the number on the received image using the above mentioned trained model (returns the number one-hot encoded)
    * converts the one-hot encoded number to the corresponding the real class number (Int32)
    * publishes the predicted number (real class number, not the one-hot encoded one) to /camera/input/specific/number 
    * subscribes to topic /camera/output/specific/check (Boolean as a indication if the predicted number was correct) to verify the prediction
    * prints outcome of verification
    
    **In case of a random number:**
    * subscribes to this topic to receive image data from /camera/output/random/compressed_img_msgs
    * subscribes to /camera/output/random/number to also receive its correct label
    * converts the CompressedImage to cv2 format
    * predicts the number on the received image using the above mentioned trained model (returns the number one-hot encoded)
    * converts the one-hot encoded number to the corresponding the real class number (Int32)
    * for verification purposes the predicted value is being saved in global scope of this class
    * verification: compares the beforehand received correct label with its own predicted value
    * prints outcome of verification

## The ROS graph (using rqt)
**rqt** The rqt_graph plugin provides introspection and visualization of a live ROS system, showing nodes and the 
connections between them, and allowing you to easily debug and understand your running system and how it is structured.

Sources: 
- http://www.ros.org/core-components/, last visited 11.11.2018
