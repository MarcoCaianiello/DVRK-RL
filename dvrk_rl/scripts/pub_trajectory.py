#!/usr/bin/env python

import rospy
import ddpg_tool as tool
import ddpg_param as prm
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def get_path (start, goal):
    path=[]
    dist = goal - start
    k = 200

    path.append(start)
    
    for i in range(1,k):
        path.append(path[i-1]+dist/k)

    path.append(goal)

    return path


if __name__ == '__main__':
    rospy.init_node('PUB_TRAJECTORY')
    tipPosDes_publisher = rospy.Publisher("/dvrk/PSM1/tip_desired_path", Path, queue_size=1)
    #path = tool.read_from_file()

    start = np.concatenate([prm.startPosition, prm.startEuler])
    goal = np.concatenate([prm.goalPosition, prm.goalEuler])

    path = get_path(start, goal)
    
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
        
        #if (data[3] <= prm.pi/2) : data[3] -= 0.1
        #if (data[4] != 1000) : data[4] += 2*prm.pi
        #if (data[5] != 1000) : data[5] += 2*prm.pi
        
        temp.pose.orientation.x = data[3]
        temp.pose.orientation.y = data[4]
        temp.pose.orientation.z = data[5]
        temp.pose.orientation.w = 0
        msg.poses.append(temp)
        print(data)
    

    while True:
        tipPosDes_publisher.publish(msg)
        rospy.sleep(2)