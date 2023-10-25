#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of Swift-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import math


class swift():
    """docstring for swift"""

    def __init__(self):

        # initializing ros node with name ne_control
        rospy.init_node('drone_control')

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        # self.drone_position = [-0.03, 0.03, 37.22]
        self.drone_position = [0.0, 0.0, 0.0]
        # [x_setpoint, y_setpoint, z_setpoint]
        # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
        # self.setpoint = [2, 2, 20]
        # self.setpoint = [0.0, 0.0, 23]
        # self.setpoint = [2, 0.0, 23]
        # self.setpoint = [2, 2, 23]
        # self.setpoint = [2, 2, 25]
        # self.setpoint = [-5, 2, 25]
        # self.setpoint[-5,-3,25]
        # self.setpoint[-5,-3,21]
        # self.setpoint[7,-3,21]
        # self.setpoint[7,0,21]
        # self.setpoint[0,0,21]
        self.setpoints = [
            [0, 0, 23],
            [2, 0, 23],
            [2, 2, 23],
            [2, 2, 25],
            [-5, 2, 25],
            [-5, -3, 25],
            [-5, -3, 21],
            [7, -3, 21],
            [7, 0, 21],
            [0, 0, 19]
        ]

        self.current_setpoint_index = 0
        self.setpoint = self.setpoints[self.current_setpoint_index]
        self.setpoint_reached = False

        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = swift_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        # initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        # after tuning and computing corresponding PID parameters, change the parameters
        # self.Kp = [44, 44, 109]
        # self.Ki = [0.66, 0.66, 8]
        # self.Kd = [109, 524, 1286]
        # self.Kp = [5, 30, 90]
        # self.Ki = [0.1, 0.66, 0.2]
        # self.Kd = [0.1, 247, 873]
        # self.Kp = [0, 0, 0]
        # self.Ki = [0, 0, 0]
        # self.Kd = [0, 0, 0]

        # self.Kp = [18.36, 4.68, 21] # prebest time 112 seconds
        # self.Ki = [0, 0, 0.218]
        # self.Kd = [109.8, 26.7, 317]
        self.Kp = [11.88, 3.6, 22.8]  # current best
        self.Ki = [0.0024, 0.002, 0.32]
        self.Kd = [13.2, 19.8, 419.1]

        # -----------------------Add other required variables for pid here ----------------------------------------------

        # ------------------------Define variables for storing error in each axis----------------------------------------------
        self.error = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        self.error_sum = [0, 0, 0]
        self.max_values = [5000, 5000, 5000]
        self.min_values = [1000, 1000, 1000]
        self.out_roll = 0
        self.out_pitch = 0
        self.out_throttle = 0

        # Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
        # self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
        # You can change the upper limit and lower limit accordingly.
        # ----------------------------------------------------------------------------------------------------------
        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        # self.sample_time = 0.060 # in seconds

        # Publishing /drone_command, /alt_error, /pitch_error, /roll_error
        self.command_pub = rospy.Publisher(
            '/drone_command', swift_msgs, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------
        self.alt_error_pub = rospy.Publisher(
            '/alt_error', Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher(
            '/pitch_error', Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher(
            '/roll_error', Float64, queue_size=1)

    # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude',
                         PidTune, self.altitude_set_pid)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        # ------------------------------------------------------------------------------------------------------------
        self.arm()  # ARMING THE DRONE

    # Disarming condition of the drone

    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    # Arming condition of the drone : Best practise is to disarm and then arm the drone.

    def arm(self):

        self.disarm()
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)  # Publishing /drone_command
        rospy.sleep(1)
    # Whycon callback function
    # The function gets executed each time when /whycon node publishes /whycon/poses

    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------

        # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_altitude
    # This function gets executed each time when /tune_pid publishes /pid_tuning_altitude

    # def altitude_set_pid(self, alt):
    #     # This is just for an example. You can change the ratio/fraction value accordingly
    #     self.Kp[2] = alt.Kp
    #     self.Ki[2] = alt.Ki * (1/1000)
    #     self.Kd[2] = alt.Kd

    # def pitch_set_pid(self, Pitch):
    #     self.Kp[1] = Pitch.Kp *0.5
    #     self.Ki[1] = Pitch.Ki * (1/1000)
    #     self.Kd[1] = Pitch.Kd *0.5

    # def roll_set_pid(self, Roll):
    #     self.Kp[0] = Roll.Kp *0.5
    #     self.Ki[0] = Roll.Ki * (1/1000)
    #     self.Kd[0] = Roll.Kd  *0.5
    def altitude_set_pid(self, alt):
        # This is just for an example. You can change the ratio/fraction value accordingly
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
    # ----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):

        # -----------------------------Write the PID algorithm here--------------------------------------------------------------
        self.error[2] = self.drone_position[2]-self.setpoint[2]
        self.error[1] = self.drone_position[1]-self.setpoint[1]
        self.error[0] = self.setpoint[0]-self.drone_position[0]
        self.tolerance_value = 0.2 

        if (
            abs(self.error[0]) < self.tolerance_value
            and abs(self.error[1]) < self.tolerance_value
            and abs(self.error[2]) < self.tolerance_value
        ):
            self.current_setpoint_index += 1
            if self.current_setpoint_index >= len(self.setpoints):
                # If all setpoints are reached, you can choose to reset or stop
                rospy.loginfo("All setpoints reached!")
                print("All setpoints reached!")

            else:
                # Switch to the next setpoint
                self.setpoint = self.setpoints[self.current_setpoint_index]
                print("Switching to next setpoint: ", self.setpoint)
                self.setpoint_reached = False

    # Rest of the pid method remains unchanged...
        # print(self.error)
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

        # print("throtel p term  ", p_term_z)
        # print("throtel d term  ", d_term_z)
        # print("throtel i term  ", i_term_z)
        # print("throtel out  ", self.out_throttle)
        # print("throtel error  ", self.error[2])
        # print("throtel prev error  ", self.prev_error[2])
        # print("throtel error sum  ", self.error_sum[2])
        # print("throtel kp  ", self.Kp[2])
        # print("throtel kd  ", self.Kd[2])
        # print("throtel ki  ", self.Ki[2])
        # print("rcthrottel ", self.cmd.rcThrottle)
        # print("throtel drone position  ", self.drone_position[2])
        # print("throtel setpoint  ", self.setpoint[2])

        # print("pitch p term  ", p_term_y)
        # print("pitch d term  ", d_term_y)
        # print("pitch i term  ", i_term_y)
        # print("pitch out  ", self.out_pitch)
        # print("pitch error  ", self.error[1])
        # print("pitch prev error  ", self.prev_error[1])
        # print("pitch error sum  ", self.error_sum[1])
        # print("pitch kp  ", self.Kp[1])
        # print("pitch kd  ", self.Kd[1])
        # print("pitch ki  ", self.Ki[1])
        # print("pitch drone position  ", self.drone_position[1])
        # print("pitch setpoint  ", self.setpoint[1])

        # print("roll p term  ", p_term_x)
        # print("roll d term  ", d_term_x)
        # print("roll i term  ", i_term_x)
        # print("roll out  ", self.out_roll)
        # print("roll error  ", self.error[0])
        # print("roll prev error  ", self.prev_error[0])
        # print("roll error sum  ", self.error_sum[0])
        # print("roll kp  ", self.Kp[0])
        # print("roll kd  ", self.Kd[0])
        # print("roll ki  ", self.Ki[0])
        # print("roll drone position  ", self.drone_position[0])
        # print("roll setpoint  ", self.setpoint[0])

        # print("SET POINt", self.setpoint[0],
        #       self.setpoint[1], self.setpoint[2])

        # print("DRONE POSITION", self.drone_position[0], self.drone_position[1],
        #       self.drone_position[2])

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

        # Steps:
        # 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
        # 2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
        # 3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
        # 4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
        # 5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
        # 6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
        # self.cmd.rcPitch = self.max_values[1]
        # 7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
        # 8. Add error_sum

        # ------------------------------------------------------------------------------------------------------------------------
        self.command_pub.publish(self.cmd)
        self.alt_error_pub.publish(self.error[2])
        self.pitch_error_pub.publish(self.error[1])
        self.roll_error_pub.publish(self.error[0])


if __name__ == '__main__':

    swift_drone = swift()
    start_time = time.time()
    # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    r = rospy.Rate(30)
    while not rospy.is_shutdown():

        swift_drone.pid()
        if swift_drone.current_setpoint_index >= len(swift_drone.setpoints):
            end_time = time.time()  # Record the end time
            print("All setpoints reached!")
            print("Total time taken:", end_time - start_time, "seconds")
            break
        r.sleep()
