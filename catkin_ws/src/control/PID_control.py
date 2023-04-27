#!/usr/bin/env python3
import rospy
import tf
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import TransformStamped
from math import pow, atan2, sqrt, pi, degrees
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
# Change the mode of drone
from mavros_msgs.srv import SetMode
# Carry out arm/disarm of drone
from mavros_msgs.srv import CommandBool
# Take off or land a drone
from mavros_msgs.srv import CommandTOL

def euclidean_distance(x_d, y_d, z_d):
    return sqrt(pow((x_d), 2) + pow((y_d), 2) + pow((z_d), 2))

class State:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class PID:
    def __init__(self, kp=1, kd=0, ki=0, dt=0.01):

        # Gains
        self.kp = kp
        self.kd = kd
        self.ki = ki

        # time step
        self.dt = dt

        # Default Error Initialization
        self.err_previous = 0.001
        self.err_accmulation = 0

    def compute(self, err):

        # compute dervivative
        err_deriv = (err - self.err_previous) / self.dt

        # update integration
        self.err_accmulation = self.err_accmulation + self.dt * (err + self.err_previous)/2

        # compute pid equation
        pid = self.kp * err + self.kd * err_deriv + self.ki * self.err_accmulation

        # update error
        self.err_previous = err
        return pid

class Controller:
    def __init__(self):
        # initialize the ros node and relevant publisher/subscriber
        rospy.init_node("PID_node")
        # Setpoint velocity of MAV. MAV can move at the desired speed
        self.velocity_publisher = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        # Target position of MAV in local coordinate frame
        self.position_publisher = rospy.Publisher("/mavros/setpoint_position//local", PoseStamped, queue_size=1)
        self.bebop_subscriber = rospy.Subscriber("/relative_distance", Float32MultiArray, self.call_back)
        # Receiving pose information of MAV in local coordinate frame
        self.position_subscriber = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pos_call_back)

        # drone current state
        self.state = State()
        
        # PoseStamped class that stores the current location of the drone
        self.local_position = PoseStamped()

        # controller frequency in Hz
        self.hz = 50.0
        # Limit the rate in which ROS nodes run
        self.rate = rospy.Rate(self.hz)
        self.dt = (1.0 / self.hz)

        # PID controller class 
        self.pid_rho = PID(kp=1, dt = self.dt)


    # function that handles messages received by the subscriber
    # Relative position to target point
    def call_back(self, msg):
        self.state.x = -msg.data[1]
        self.state.y = -msg.data[0]
        self.state.z = -msg.data[2]
    
    # Store location information on the drone's local coordinate system
    def pos_call_back(self, msg):
        self.local_position.pose.position.x = msg.pose.position.x
        self.local_position.pose.position.y = msg.pose.position.y
        self.local_position.pose.position.z = msg.pose.position.z

    def takeoff(self):
        print("Fly to z=2")
        pos_msg = PoseStamped()
        pos_msg.pose.position.x = 0
        pos_msg.pose.position.y = 0
        pos_msg.pose.position.z = 2
        for i in range(200):
            self.position_publisher.publish(pos_msg)
            self.rate.sleep()
     
    def move_to_goal(self):
        # variable initialization
        vel_msg = Twist()
        
        # Allowance to reach target position
        tolerance_position = 0.01
        
        # Calculate the distance between the current position and the target position
        rho = euclidean_distance(self.state.x, self.state.y, self.state.z)
        
        while (rho >= tolerance_position or rho ==0): # and self.state.z < -1
            rospy.loginfo("Distance form goal:" + str(rho))

            rho = euclidean_distance(self.state.x, self.state.y, self.state.z)
            err_x = self.state.x
            err_y = self.state.y
            err_z = self.state.z

            # compute PID
            vx = self.pid_rho.compute(err_x)
            vy = self.pid_rho.compute(err_y)
            vz = self.pid_rho.compute(err_z) *0.25

            # fill message
            vel_msg.linear.x = vx
            vel_msg.linear.y = vy
            vel_msg.linear.z = vz
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0

            # debuggin
            print("vx: {}".format(vel_msg.linear.x))
            print("x: {}".format(self.state.x))
            print("vy: {}".format(vel_msg.linear.y))
            print("y: {}".format(self.state.y))
            print("vz: {}".format(vel_msg.linear.z))
            print("z: {}".format(self.state.z))
            print("_________________")

            # publish
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        if self.local_position.pose.position.z > 0:
            print("landing begin")
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = -1.0

            for i in range(1000):
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()

        # stop the drone
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        self.velocity_publisher.publish(vel_msg)

        #Land
        print("\n Landing")
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            land_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            response = land_cl(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Landing failed: %s" %e)

        # Disarm
        print("\n Disarming")
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arming_cl(value = False)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Disarming failed: %s" %e)

        rospy.loginfo(
            "I'm here(relative info): " + str(self.state.x) + " , " + str(self.state.y) + " , " + str(self.state.z))
        print("___")

        return
if __name__ == "__main__":
    try:
        rospy.wait_for_service("/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        x = Controller()

        #x.takeoff()

        x.move_to_goal()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

