import os
import sys
import cv2
import numpy as np
import tensorflow as tf
import rospy
from styx_msgs.msg import TrafficLight

BASE_DIR = '../../../data/'

class TLClassifier(object):
    def __init__(self, is_sim):
        if is_sim:
            MODEL_DIR = BASE_DIR + 'frozen_model_sim/'
        else:
            MODEL_DIR = BASE_DIR + 'frozen_model_real/'

        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        PATH_TO_CKPT = MODEL_DIR + 'frozen_inference_graph.pb'
        self.graph = tf.Graph()
        self.threshold = 0.6
        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                od_graph_def.ParseFromString(fid.read())
                tf.import_graph_def(od_graph_def, name='')

            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name(
                'num_detections:0')
        self.sess = tf.Session(graph=self.graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        #rospy.logwarn('image_shape {0}'.format(image.shape))
        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_expand})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        if scores[0] > self.threshold:
            if classes[0] == 1:
                rospy.logwarn('GREEN')
                return TrafficLight.GREEN
            elif classes[0] == 2:
                rospy.logwarn('RED')
                return TrafficLight.RED
            elif classes[0] == 3:
                rospy.logwarn('YELLOW')
                return TrafficLight.YELLOW
        rospy.logwarn('UNKNOWN')
        return TrafficLight.UNKNOWN
