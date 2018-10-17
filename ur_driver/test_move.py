#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from ur_driver.io_interface import *

PI=3.14159265359
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [2.2,0,-1.57,0,0,0]
Q2 = [138,-0,0,-0,0,0]
Q3 = [188,-0,0,-0,0,0]
HOME_VIEW = [0,-0,0,-0,20,0]
HOME_STORE = [1.5,-0.2,-1.57,0,0,0]
HOME_DRIVE = [1.5,-0.2,-1.57,0,0,0]
GRIP=2
OPEN=3

client = None

def moveto(Q):
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q, velocities=[0]*6, time_from_start=rospy.Duration(400.0)),

        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(800.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(1600.0))]
	
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_disordered():
    order = [4, 2, 3, 1, 5, 0]
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = [JOINT_NAMES[i] for i in order]
    q1 = [Q1[i] for i in order]
    q2 = [Q2[i] for i in order]
    q3 = [Q3[i] for i in order]
    g.trajectory.points = [
        JointTrajectoryPoint(positions=q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    client.send_goal(g)
    client.wait_for_result()
    
def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    
    d = 2.0
    g.trajectory.points = []
    for i in range(10):
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 1
        g.trajectory.points.append(
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

def move_interrupt():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
    
    client.send_goal(g)
    time.sleep(2.0)
    print "Interrupting"
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
def detection_client():
    client = actionlib.SimpleActionClient("detection", edge_detection_pmd.msg.DetectionAction)
    client.wait_for_server()
    goal = edge_detection_pmd.msg.DetectionGoal(numberOfFrames = 1)
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


def main():
    global client
    try:




        #rospy.init_node('detection_client')
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"

#Actionservers are activated and ready

#Move-trajectory: HOME_VIEW - p_pick - HOME_DRIVE - HOME_STORE

	#set_digital_out(OPEN, False)
	#set_digital_out(GRIP, False)
        moveto(HOME_VIEW)
	input("ENTER")
	result = detection_client()
	p_pick= HOME_VIEW
	#p_pick[0]= p_pick[0]-result.midpoint_position[0]*100
	#p_pick[1]= p_pick[1]-result.midpoint_position[1]*100
	p_pick[2]= p_pick[2]-result.midpoint_position[2]*100
	#p_pick[5]= p_pick[5]-result.rz
	moveto(p_pick)
	set_digital_out(OPEN, False)
	set_digital_out(GRIP, True)
	time.sleep(1)
	input("ENTER")
	moveto(HOME_DRIVE)
	input("ENTER")
	moveto(HOME_STORE)
	set_digital_out(GRIP, False)
	set_digital_out(OPEN, True)
        #move_repeated()
        #move_disordered()
        #move_interrupt()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
