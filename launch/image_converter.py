#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

roslib.load_manifest('enph353_ros_lab')


class image_converter:
    """
    A class that converts ROS image messages to OpenCV format and
        processes them.

    This class subscribes to a ROS topic for raw image data, processes the
        image to detect
    circles, and publishes robot movement commands based on the detected
        circle's position.
    """

    def __init__(self):
        """
        Initializes the image_converter class.

        Sets up image publisher and subscriber, and initializes the CvBridge.
        """
        self.image_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image,
                                          self.callback)
        self.constant = 0

    def callback(self, data):
        """
        Callback function to process incoming image data.

        Converts the ROS Image message to a CV2 image and
            processes it to find circles.
        Calculates movement commands based on the circle's
            position relative to the image center.

        :param data: The incoming ROS Image message.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        height, width = cv_image.shape[:2]

        # creates movement object to publish to
        move = Twist()
        rate = rospy.Rate(30)  # set rate to 30Hz

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        circle_left = 0
        circle_right = 0
        crossed_in = False
        crossed_out = True

        # iterates through the gray scale images
        for index, value in enumerate(gray[-1]):
            if value < 150 and not crossed_in:
                circle_left = index
                crossed_in = True
                crossed_out = False
            if value > 150 and not crossed_out:
                circle_right = index
                crossed_out = True

        # find center and comparison values
        center = int((circle_right - circle_left) / 2) + circle_left
        comp_val = int(width / 2)

        if (center < comp_val):
            move.linear.x = 0.25
            if gray[-50][150] < 150:
                # if forward component on left side is gray, less left turn
                move.angular.z = 1.0
            else:
                move.angular.z = 3.0 * np.abs(center - comp_val) / comp_val
        elif (center > comp_val):
            move.linear.x = 0.25
            if gray[-200][650] < 150:
                # if forward component on right side is gray, more right turn
                move.angular.z = -5.5
            else:
                move.angular.z = -4.5 * np.abs(center - comp_val) / comp_val
        # Edge case for bend when path extends across screen
        if (circle_left > comp_val and circle_right < comp_val):
            move.angular.z = -7.0

        # Draw circle for testing
        center_pt = (int((circle_right - circle_left)/2) + circle_left,
                     height - 50)
        radius = 20
        color = (0, 0, 0)
        line_thickness = -1
        cv2.circle(gray, center_pt, radius, color, line_thickness)

        cv2.imshow("Image window", gray)
        cv2.waitKey(3)

        self.image_pub.publish(move)
        rate.sleep()

        # ensure speed doesn't increase for accuracy
        move.linear.x = 0.12
        if (center < comp_val):
            move.angular.z = 0.03
        else:
            move.angular.z = -0.06
        if (circle_left > comp_val and circle_right < comp_val):
            move.angular.z = -7.0
        self.image_pub.publish(move)
        rate.sleep()


def main(args):
    """
    Main function to start the image converter node.

    Initializes the ROS node and starts the main
        loop to process incoming images.

    :param args: Command line arguments.
    """
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
