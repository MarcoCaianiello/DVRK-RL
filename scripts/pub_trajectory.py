#!/usr/bin/env python

import rospy
import ddpg_tool as tool
import ddpg_param as prm
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def get_path (start, goal, n):
    path=[]
    dist = goal - start
    k = n

    path.append(start)
    
    for i in range(1,k):
        path.append(path[i-1]+dist/k)

    path.append(goal)

    return path

def get_msg (path):

    msg = Path()
    msg.header.frame_id = "end_effector"
    msg.header.stamp = rospy.Time.now()

    for i in range(len(path)):
        for j in range(len(path[i])):
            path[i][j] = round(path[i][j], 4)
    
    for data in path:
        temp = PoseStamped()
        
        temp.pose.position.x = data[0]
        temp.pose.position.y = data[1]
        temp.pose.position.z = data[2]
        
        temp.pose.orientation.x = data[3]
        temp.pose.orientation.y = data[4]
        temp.pose.orientation.z = data[5]
        temp.pose.orientation.w = 0
        msg.poses.append(temp)
        print(data)
    
    while True:
        tipPosDes_publisher.publish(msg)
        rospy.sleep(2)

def get_demostration_buffer(path):
    capacity = len(path)-1
    state_buffer = np.zeros((capacity, prm.num_states))
    action_buffer = np.zeros((capacity, prm.num_actions))
    reward_buffer = np.zeros((capacity, 1))
    next_state_buffer = np.zeros((capacity, prm.num_states))

    for i in range(capacity):
        state_buffer[i] = path[i]
        action_buffer[i] = path[i+1] - path[i]
        next_state_buffer[i] = path[i+1]
    
    reward_buffer[capacity-1] = 1

    '''
    print(state_buffer[capacity-1])
    print(action_buffer[capacity-1])
    print(reward_buffer[capacity-1])
    print(next_state_buffer[capacity-1])

    print(path[len(path)-1])
    '''

    return state_buffer, action_buffer, reward_buffer, next_state_buffer

if __name__ == '__main__':
    rospy.init_node('PUB_TRAJECTORY')
    tipPosDes_publisher = rospy.Publisher("/dvrk/PSM1/tip_desired_path", Path, queue_size=1)
    #path = tool.read_from_file()

    start = np.concatenate([prm.startPosition, prm.startEuler])
    goal = np.concatenate([prm.goalPosition, prm.goalEuler])
    
    state_buffer = []
    action_buffer = []
    reward_buffer = []
    next_state_buffer = []

    n = [30, 50, 80, 100]
    for k in n:
        l1, l2, l3, l4 = get_demostration_buffer(get_path(start, goal, k))
        for i in range(k-1):
            state_buffer.append(l1[i])
            action_buffer.append(l2[i])
            reward_buffer.append(l3[i])
            next_state_buffer.append(l4[i])
    
    tool.save_on_file(state_buffer, "state_buffer.txt")
    tool.save_on_file(action_buffer, "action_buffer.txt")
    tool.save_on_file(reward_buffer, "reward_buffer.txt")
    tool.save_on_file(next_state_buffer, "next_state_buffer.txt")
    
    #get_msg(path)

    

