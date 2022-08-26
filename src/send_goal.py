#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

import serial

def initConnection(portNo, baudRate):
    try:
        ser = serial.Serial(
                portNo,
                baudRate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,)
        print("Device Connected")
        return ser
    except:
        print("Not Connected")

def movebase_client(pos, quat):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")

    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        ser = initConnection(portNo="/dev/ttyS0", baudRate=115200)
        rospy.init_node('movebase_client_py')
        while True:
            #data=ser.readline()
            ser.flushOutput()
            data=ser.read()
            print(data)
            if data[0] == '1':
                position = {'x': 0.5, 'y' : 0.5}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
                rospy.loginfo("Go to (%s, %s) pose[0]", position['x'], position['y'])
                result = movebase_client(position, quaternion)
            elif data == '2':
                position = {'x': 0.5, 'y' : 0.5}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
                rospy.loginfo("Go to (%s, %s) pose1", position['x'], position['y'])
                result = movebase_client(position, quaternion)
            elif data == '3':
                position = {'x': 0.5, 'y' : 0.5}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
                rospy.loginfo("Go to (%s, %s) pos--e", position['x'], position['y'])
                result = movebase_client(position, quaternion)
            elif data == '4':
                position = {'x': 0.5, 'y' : 0.5}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                result = movebase_client(position, quaternion)
            else:
                rospy.loginfo("I Don't Know")
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
                                                              
