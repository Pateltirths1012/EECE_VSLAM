#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from rospy.client import set_param
from rospy.exceptions import ROSException
import math


class mission_control:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('mission_control', anonymous=True)

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

    def offboard_set_mode(self):
        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        rospy.wait_for_service('mavros/set_mode')
        try:
            offset = rospy.ServiceProxy('mavros/set_mode',mavros_msgs.srv.SetMode)
            offset(5,"OFFBOARD")
        except rospy.ServiceException as e:
            print("Servive set_mode call failed: %s"%e)

    
    def land_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            offset = rospy.ServiceProxy('mavros/set_mode',mavros_msgs.srv.SetMode)
            offset(0,"AUTO.LAND")
        except rospy.ServiceException as e:
            print("Service set_mode call failed: %s"%e)


class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message

    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers    
    def local_Position_Cb(self, data):
        self.local_position = data

    def local_velocity_Cb(self, data):
        self.local_velocity = data

stateMt = stateMoniter()
ofb_ctl = mission_control()
pos = PoseStamped()
vel = Twist()
local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(20)

def yaw_to_quaternion(yaw):
    yaw_rad = math.radians(yaw)
    qw = math.cos(yaw_rad / 2)
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw_rad / 2)
    return [qx, qy, qz, qw]

# Function to hover at a certain position in offboard mode
def hover():
    for i in range (100):
        local_pos_pub.publish(pos)
        rate.sleep()

def main():

    pos.pose.position.x = 1
    pos.pose.position.y = 0
    pos.pose.position.z = 2

    vel.linear.x = 1
    vel.linear.y = 0
    vel.linear.z = 0

    wayPoint1 = False
    wayPoint2 = False
    wayPoint3 = False
    wayPoint4 = False
    wayPoint5 = False
    wayPoint6 = False
    wayPoint7 = False
    wayPoint8 = False
    wayPoint9 = False

    rospy.Subscriber("/mavros/state", State, stateMt.stateCb)

    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, stateMt.local_Position_Cb)

    rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, stateMt.local_velocity_Cb )

    for i in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()

    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")

    while not stateMt.state.mode=="OFFBOARD":
        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print("OFFBOARD mode activated")

    while not rospy.is_shutdown():

        local_pos_pub.publish(pos)
        local_vel_pub.publish(vel)

        if (abs(2 - stateMt.local_position.pose.position.z) < 1) and (wayPoint1==False):
            hover()
            Quaternion = yaw_to_quaternion(-90)
            pos.pose.orientation.x = Quaternion[0]
            pos.pose.orientation.y = Quaternion[1]
            pos.pose.orientation.z = Quaternion[2]
            pos.pose.orientation.w = Quaternion[3]
            pos.pose.position.y = -3
            wayPoint1 = True

        if (abs(3 + stateMt.local_position.pose.position.y) < 1) and (wayPoint2==False) and (wayPoint1 == True):
            hover()
            Quaternion = yaw_to_quaternion(-180)
            pos.pose.orientation.x = Quaternion[0]
            pos.pose.orientation.y = Quaternion[1]
            pos.pose.orientation.z = Quaternion[2]
            pos.pose.orientation.w = Quaternion[3]
            pos.pose.position.x = -6
            wayPoint2 = True

        if (abs(6 + stateMt.local_position.pose.position.x) < 1) and (wayPoint3 == False) and (wayPoint2 == True):
            hover()
            Quaternion = yaw_to_quaternion(-270)
            pos.pose.orientation.x = Quaternion[0]
            pos.pose.orientation.y = Quaternion[1]
            pos.pose.orientation.z = Quaternion[2]
            pos.pose.orientation.w = Quaternion[3]
            pos.pose.position.y = 3
            wayPoint3 = True

        if ((3 - stateMt.local_position.pose.position.y) < 1) and wayPoint4 == False and wayPoint3 == True:
            hover()
            Quaternion = yaw_to_quaternion(-360)
            pos.pose.orientation.x = Quaternion[0]
            pos.pose.orientation.y = Quaternion[1]
            pos.pose.orientation.z = Quaternion[2]
            pos.pose.orientation.w = Quaternion[3]
            pos.pose.position.y = -3
            wayPoint4 = True

        if (abs(3 + stateMt.local_position.pose.position.y) < 1) and wayPoint5 == False and wayPoint4 == True: 
            hover()
            Quaternion = yaw_to_quaternion(0)
            pos.pose.orientation.x = Quaternion[0]
            pos.pose.orientation.y = Quaternion[1]
            pos.pose.orientation.z = Quaternion[2]
            pos.pose.orientation.w = Quaternion[3]
            pos.pose.position.x = 6
            wayPoint5 = True
    
        if ((6 - stateMt.local_position.pose.position.x) < 1) and wayPoint6 == False and wayPoint5 == True:
            hover()
            Quaternion = yaw_to_quaternion(90)
            pos.pose.orientation.x = Quaternion[0]
            pos.pose.orientation.y = Quaternion[1]
            pos.pose.orientation.z = Quaternion[2]
            pos.pose.orientation.w = Quaternion[3]
            pos.pose.position.y = 1
            wayPoint6 = True

        if ((1 - stateMt.local_position.pose.position.y) < 1) and wayPoint7 == False and wayPoint6 == True:
            hover()
            Quaternion = yaw_to_quaternion(180)
            pos.pose.orientation.x = Quaternion[0]
            pos.pose.orientation.y = Quaternion[1]
            pos.pose.orientation.z = Quaternion[2]
            pos.pose.orientation.w = Quaternion[3]
            pos.pose.position.x = 1
            wayPoint7 = True
        
        if ((1 - stateMt.local_position.pose.position.x) < 1) and wayPoint8 == False and wayPoint7 == True:
            hover()
            Quaternion = yaw_to_quaternion(270)
            pos.pose.orientation.x = Quaternion[0]
            pos.pose.orientation.y = Quaternion[1]
            pos.pose.orientation.z = Quaternion[2]
            pos.pose.orientation.w = Quaternion[3]
            pos.pose.position.y = 0
            wayPoint8 = True

        # if (1 - (stateMt.local_position.pose.position.y) < 1) and ((1 - stateMt.local_position.pose.position.x) < 1) and wayPoint9 == False and wayPoint8 == True:
        #     # hover()
        #     Quaternion = yaw_to_quaternion(0)
        #     pos.pose.orientation.x = Quaternion[0]
        #     pos.pose.orientation.y = Quaternion[1]
        #     pos.pose.orientation.z = Quaternion[2]
        #     pos.pose.orientation.w = Quaternion[3]
        #     ofb_ctl.land_mode()
        #     wayPoint9 = True


        rate.sleep()
    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
