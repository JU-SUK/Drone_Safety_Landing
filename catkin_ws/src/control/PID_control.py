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
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
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
        self.err_acc = 0

    def comput(self, err):

        # compute dervivative
        err_deriv = (err - self.err_previous) / self.dt

        # update integration
        self.err_acc = self.err_acc + self.dt * (err + self.err_previous)/2

        # compute pid equation
        pid = self.kp * err + self.kd * err_deriv + self.ki * err_acc

        # update error
        self.err_previous = err

        return pid

class Controller:
    def __init__(self):
        # initialize the ros node and relevant publisher/subscriber
        rospy.init_node("PID_node")
        self.velocity_publisher = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.position_publisher = rospy.Publisher("/mavros/setpoint_position//local", PoseStamped, queue_size=1)
        self.bebop_subscriber = rospy.Subscriber("/relative_distance", Float32MultiArray, self.call_back)
        self.position_subscriber = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pos_call_back)

        # robot current state
        self.state = State()

        self.local_position = PoseStamped()

        # controller frequency in Hz
        self.hz = 50.0
        self.rate = rospy.Rate(self.hz)
        self.dt = (1.0 / self.hz)

        # define pids
        self.pid_rho = PID(kp=0.2, dt = self.dt)


    # tranformation
    def call_back(self, msg):
        self.state.x = -msg.data[1]
        self.state.y = -msg.data[0]
        self.state.z = -msg.data[2]

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
        tolerance_position = 0.01

        rho = euclidean_distance(self.state.x, self.state.y, self.state.z)
        while (rho >= tolerance_position or rho ==0) and self.state.z < -1:
            rospy.loginfo("Distance form goal:" + str(rho))

            rho = euclidean_distance(self.state.x, self.state.y, self.state.z)
            err_x = self.state.x
            err_y = self.state.y
            err_z = self.state.z

            # compute PID
            vx = self.pid_rho.compute(err_x)
            vy = self.pid_rho.compute(err_y)
            vz = self.pid_rho.compute(err_z) * 0.25

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
            vel_msg.linear.z = -2.0

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
            response = land_cl(altitude=, latitude=0, longitude=0, min_pitch=0, yaw=0)
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
        x = Controller()

        x.takeoff()

        x.move_to_goal()

        rospy.spin()

    except rospy.ROSInterrupException:
        pass

