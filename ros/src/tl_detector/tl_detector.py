#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight, Waypoint
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import yaml
import sys
import math
from timeit import default_timer as timer

UNKNOWN = -1
MAX_DISTANCE = sys.maxint
STATE_COUNT_THRESHOLD = 3
FREQUENCY_IN_HERTZ = 10.0


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        rospy.logwarn("### Traffic Light Detector Initialization ....")

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.image_processing_time = 0

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        # Skip Traffic Light Detection if processing takes longer than current frequency, avoid queueing
        if self.image_processing_time > 0:
            self.image_processing_time -= self.hertz2Seconds(FREQUENCY_IN_HERTZ)
            # rospy.logwarn("Skipping traffic light detection for this frame ...: %f s", self.image_processing_time)
            return

        self.has_image = True
        self.camera_image = msg
        start_detection = timer()
        light_wp, state = self.process_traffic_lights()
        end_detection = timer()
        self.image_processing_time = end_detection - start_detection

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state

            if state == TrafficLight.RED or state == TrafficLight.YELLOW:
                light_wp = light_wp
            else:
                light_wp = UNKNOWN

            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if (not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        predicted = self.light_classifier.get_classification(cv_image)

        rospy.logdebug("traffic light state: %d", light.state)

        return predicted

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        light = None
        distance_tolerance = 300
        stop_line_position = None

        # List of positions that correspond to the line to stop in front of for a given intersection

        if self.pose:
            car_index = self.get_closest_waypoint(self.pose.pose.position)
            car_position = self.waypoints.waypoints[car_index].pose.pose.position
            # rospy.loginfo("Car Position:{}".format(car_position))

            light_index = self.get_closest_light(car_position)
            if light_index != UNKNOWN:
                light_waypoint_index = self.get_closest_waypoint(self.lights[light_index].pose.pose.position)
                light_position = self.waypoints.waypoints[light_waypoint_index].pose.pose.position
                # rospy.loginfo("Nearest Light Position:{}".format(light_position))

                if light_waypoint_index > car_index:
                    distance_to_traffic_light = self.distance_of_positions(car_position, light_position)
                    if distance_to_traffic_light < distance_tolerance:
                        light = self.lights[light_index]
                        stop_line_index = self.get_closest_stop_line(light_position)
                        stop_line_position = self.get_stop_line_positions()[stop_line_index].pose.pose
                        stop_line_waypoint = self.get_closest_waypoint(stop_line_position.position)
                        # rospy.loginfo("Nearest Stop Line within Range:{}".format(stop_line_position))

        if light and stop_line_position:
            # if int(distance_to_traffic_light) % 5 == 0:
            #    rospy.logwarn("### Traffic Light Detected in: %.0f m", distance_to_traffic_light)
            state = self.get_light_state(light)
            return stop_line_waypoint, state

        return -1, TrafficLight.UNKNOWN

    # Helper Methods

    def hertz2Seconds(self, hertz):
        return hertz / 100

    def get_closest_waypoint(self, pose):
        return self.get_closest_index(pose, self.waypoints.waypoints)

    def get_closest_light(self, pose):
        return self.get_closest_index(pose, self.lights)

    def get_closest_stop_line(self, pose):
        return self.get_closest_index(pose, self.get_stop_line_positions())

    def get_stop_line_positions(self):
        stop_line_positions = []
        for light_position in self.config['stop_line_positions']:
            p = Waypoint()
            p.pose.pose.position.x = light_position[0]
            p.pose.pose.position.y = light_position[1]
            p.pose.pose.position.z = 0.0
            stop_line_positions.append(p)
        return stop_line_positions

    def get_closest_index(self, pose, positions):
        minimal_distance = MAX_DISTANCE
        index = UNKNOWN

        for i in range(len(positions)):
            distance = self.distance_of_positions(pose, positions[i].pose.pose.position)
            if distance < minimal_distance:
                minimal_distance = distance
                index = i

        return index

    def distance_of_positions(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2)


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
