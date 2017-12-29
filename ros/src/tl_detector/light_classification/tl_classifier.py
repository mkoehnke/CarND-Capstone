from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import os
import cv2
import rospy

MODEL_PATH = os.path.dirname(
    os.path.realpath(__file__)) + '/../../../../data/traffic_light_model/tl_frozen_inference_graph.pb'
LABEL_PATH = os.path.dirname(os.path.realpath(__file__)) + '/../../../../data/traffic_light_model/tl_label_map.pbtxt'
IMAGE_WIDTH = 300
IMAGE_HEIGHT = 300


class TLClassifier(object):
    def __init__(self):
        self.model_graph = None
        self.session = None
        self.classes = {1: TrafficLight.RED,
                        2: TrafficLight.YELLOW,
                        3: TrafficLight.GREEN,
                        4: TrafficLight.UNKNOWN}
        self.load_model()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        class_index, probability = self.predict(image)
        if class_index is not None:
            rospy.logdebug("class: %d, probability: %f", class_index, probability)
        return class_index

    def load_model(self):
        config = tf.ConfigProto()
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1

        self.model_graph = tf.Graph()
        with tf.Session(graph=self.model_graph, config=config) as sess:
            self.session = sess
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(MODEL_PATH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

    def predict(self, image_np, min_score_thresh=0.5):
        image_tensor = self.model_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = self.model_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = self.model_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.model_graph.get_tensor_by_name('detection_classes:0')
        image_np = cv2.resize(image_np, (IMAGE_WIDTH, IMAGE_HEIGHT))
        image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

        (boxes, scores, classes) = self.session.run(
            [detection_boxes, detection_scores, detection_classes],
            feed_dict={image_tensor: np.expand_dims(image_np, axis=0)})

        scores = np.squeeze(scores)
        classes = np.squeeze(classes)
        boxes = np.squeeze(boxes)

        for i, box in enumerate(boxes):
            if scores[i] > min_score_thresh:
                light_class = self.classes[classes[i]]
                rospy.logdebug("Traffic Light Class detected: %d", light_class)
                return light_class, scores[i]

        #rospy.logdebug("Traffic Light Class not detected")

        return None, None
