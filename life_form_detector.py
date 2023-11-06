#!/usr/bin/env python3

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np
from imutils import contours
from skimage import measure
import numpy as np
from imutils import *

class swift():
    """docstring for swift"""

    def __init__(self):

        rospy.init_node('drone_control')

        self.drone_position = [0.0, 0.0, 0.0]
        self.setpoints = [[1, -1, 23], [3, -3, 23], [6, -4, 23], [7, -7, 23]]
        # self.setpoints = [[1, 1, 23], [3, 3, 23], [6, 4, 23], [7, 7, 23]]
        # self.setpoints = [[-1, 1, 23], [-3, 3, 23], [-6, 4, 23], [-7, 7, 23]]
        # self.setpoints = [[-1, -1, 23], [-3, -3, 23], [-6, -4, 23], [-7, -7, 23]]

        self.current_setpoint_index = 0
        self.setpoint = self.setpoints[self.current_setpoint_index]
        self.setpoint_reached = False

        self.cmd = swift_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        self.Kp = [11.88, 3.6, 22.8]  
        self.Ki = [0.0024, 0.002, 0.32]
        self.Kd = [13.2, 19.8, 419.1]

        self.error = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        self.error_sum = [0, 0, 0]
        self.max_values = [5000, 5000, 5000]
        self.min_values = [1000, 1000, 1000]
        self.out_roll = 0
        self.out_pitch = 0
        self.out_throttle = 0

        self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
        self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
        self.image_pub = rospy.Publisher('/image', Int64, queue_size=1)
        self.location_pub = rospy.Publisher('/astrobiolocation', Float64, queue_size=1)
        
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude',PidTune, self.altitude_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        rospy.Subscriber("/swift/camera_rgb/image_raw", Image, self.callback)

        self.arm()  

    def callback(self, data):
        cv2_img = CvBridge().imgmsg_to_cv2(data, "bgr8")
        img_data = np.array(cv2_img, dtype=np.uint8)
        allien_found_flag = False
        gray = cv2.cvtColor(img_data, cv2.COLOR_BGR2GRAY)
        cv2.waitKey(3)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        img_with_contours = img_data.copy()  
        curren_pos = [round(self.drone_position[0]), round(self.drone_position[1]),round(self.drone_position[2])]
        for i, contour in enumerate(contours):
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            cv2.drawContours(img_with_contours, [contour], -1, (0, 0, 255), 2)
            cv2.circle(img_with_contours, (cX, cY), 7, (0, 255, 0), -1)
            cv2.putText(img_with_contours, f"Centroid #{i + 1}", (cX - 20, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            print("Contour coordinates: ", cX, cY)

        if len(contours) >=3 or len(contours) == 1: 
            allien_count = len(contours)
            print("Allien found at: ", self.setpoint, "with count: ", allien_count)
            image_filename = f"contour_image_with_multiple_contours.jpg"
            cv2.imwrite(image_filename, img_with_contours)

    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    def arm(self):
        self.disarm()
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd) 
        rospy.sleep(1)
    
    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        print("Drone position: ", msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z)

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.Kp * 0.06
        self.Ki[2] = alt.Ki * 0.0008
        self.Kd[2] = alt.Kd * 0.3

    def pitch_set_pid(self, Pitch):
        self.Kp[1] = Pitch.Kp * 0.06
        self.Ki[1] = Pitch.Ki * 0.0008
        self.Kd[1] = Pitch.Kd * 0.3
        
    def roll_set_pid(self, Roll):
        self.Kp[0] = Roll.Kp * 0.06
        self.Ki[0] = Roll.Ki * 0.0008
        self.Kd[0] = Roll.Kd * 0.3

    def pid(self):
        self.error[2] = self.drone_position[2]-self.setpoint[2]
        self.error[1] = self.drone_position[1]-self.setpoint[1]
        self.error[0] = self.setpoint[0]-self.drone_position[0]
        self.tolerance_value =0.2
        if (
            abs(self.error[0]) < self.tolerance_value
            and abs(self.error[1]) < self.tolerance_value
            and abs(self.error[2]) < self.tolerance_value
        ):
            self.current_setpoint_index += 1
            if self.current_setpoint_index >= len(self.setpoints):
                rospy.loginfo("All setpoints reached!")
                print("All setpoints reached!")
            else:
                self.setpoint = self.setpoints[self.current_setpoint_index]
                print("Switching to next setpoint: ", self.setpoint)
                self.setpoint_reached = False
    

        p_term_x = self.Kp[0] * self.error[0]
        p_term_y = self.Kp[1] * self.error[1]
        p_term_z = self.Kp[2] * self.error[2]

        d_term_x = self.Kd[0] * (self.error[0] - self.prev_error[0])
        d_term_y = self.Kd[1] * (self.error[1] - self.prev_error[1])
        d_term_z = self.Kd[2] * (self.error[2] - self.prev_error[2])

        i_term_x = (self.Ki[0] * self.error_sum[0])
        i_term_y = (self.Ki[1] * self.error_sum[1])
        i_term_z = (self.Ki[2] * self.error_sum[2])

        self.out_roll = p_term_x + d_term_x + i_term_x
        self.out_pitch = p_term_y + d_term_y + i_term_y
        self.out_throttle = p_term_z + d_term_z + i_term_z

        self.cmd.rcRoll = int(1500 + self.out_roll)
        self.cmd.rcPitch = int(1500 + self.out_pitch)
        self.cmd.rcThrottle = int(1500 + self.out_throttle)

        self.prev_error[0] = self.error[0]
        self.prev_error[1] = self.error[1]
        self.prev_error[2] = self.error[2]

        self.error_sum[0] += self.error[0]
        self.error_sum[1] += self.error[1]
        self.error_sum[2] += self.error[2]

        if self.cmd.rcRoll > self.max_values[0]:
            self.cmd.rcRoll = self.max_values[0]
        elif self.cmd.rcRoll < self.min_values[0]:
            self.cmd.rcRoll = self.min_values[0]

        if self.cmd.rcPitch > self.max_values[1]:
            self.cmd.rcPitch = self.max_values[1]
        elif self.cmd.rcPitch < self.min_values[1]:
            self.cmd.rcPitch = self.min_values[1]

        if self.cmd.rcThrottle > self.max_values[2]:
            self.cmd.rcThrottle = self.max_values[2]
        elif self.cmd.rcThrottle < self.min_values[2]:
            self.cmd.rcThrottle = self.min_values[2]

        self.command_pub.publish(self.cmd)
        self.alt_error_pub.publish(self.error[2])
        self.pitch_error_pub.publish(self.error[1])
        self.roll_error_pub.publish(self.error[0])

        self.command_pub.publish(self.cmd)
        self.alt_error_pub.publish(self.error[2])
        self.pitch_error_pub.publish(self.error[1])
        self.roll_error_pub.publish(self.error[0])
        
if __name__ == '__main__':
    swift_drone = swift()
    start_time = time.time()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        swift_drone.pid()
        if swift_drone.current_setpoint_index >= len(swift_drone.setpoints):
            end_time = time.time() 
            print("All setpoints reached!")
            print("Total time taken:", end_time - start_time, "seconds")
            break
        r.sleep()
