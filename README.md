# Robotik KI Exercise:
## ROS (Robot Operating System) + Supervised Learning
This is an excercise for the course AI in Robotics at University of Applied Science (HTW) Berlin.

## Topics 

### Supervised Learning
Training of a model with a CNN (Convolutional Neural Network) on the MNIST dataset, using the Keras API with TensorFlow as backend

### Publisher/Subscriber in ROS 
* CameraPseudo uses MNIST data as pseudo webcam images
* Prediction 
    * includes the above mentioned trained model and predicts the value based on the subscribed image inside the subscribers callback 
    * publishes the predicted number (real class number, not the one-hot encoded one) to /camera/input/specific/number 
    * subscribes to topic /camera/output/specific/check to verify the prediction
