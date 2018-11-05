#!/usr/bin/env python
import rospy
import actionlib
import time
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


client = None
target_points = [
    (-4.31, 2.22), # Near the white board/charging spot
    (1.5,   2.00), # Facing the door
    (4.16,  5.74), # Left part of corridor after exiting lab
    (4.39,  -2.78) # Right part of corridor after exiting lab
]

sit = 0
status = 1
def goalStatus(array):
    global status
    status = array.status_list[-1].status
    if status == 4:
        sit = 1

def create_goal():
    goal = MoveBaseGoal()
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    return goal

def next_goal(i, goal):
    global client
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = target_points[i]
    rospy.loginfo(target_points[i])
    client.send_goal(goal)
    resp = client.wait_for_result()
    time.sleep(5)
    if not resp:
        rospy.loginfo("something failed")
        exit()
    else: return client.get_result()

def init():
    global client
    rospy.init_node('auto_navigation', anonymous=True)
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    rospy.Subscriber('/move_base/status', GoalStatusArray, goalStatus)
    goal = create_goal()
    i = 0
    while i < 4:
        next_goal(i, goal)
        if i == 2 and status == 4: #status 4 means plan failed. 
            rospy.loginfo('please open the door!')
        elif i == 2 and status != 4: #other status means goal must be reached because next_goal returns after full success or full failure
            rospy.loginfo('thank you for keeping the door open!')
            i = i+1
        else:
            i = i+1
    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException: pass
