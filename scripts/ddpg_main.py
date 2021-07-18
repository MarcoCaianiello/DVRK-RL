#!/usr/bin/env python3

import rospy
import tensorflow as tnf
from tensorflow.keras import layers
import numpy as np
import math

import ddpg_param as prm
import ddpg_tool as tool
import ddpg_actor_critic_policy as acp
import ddpg_buffer as buffer

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import Pose

'''
DEMOSTRATION BUFFER
Er - Et
'''

class ddpg:

    def __init__(self):
        print("SERGIO")
        self.robot_subscriber = rospy.Subscriber("/dvrk/PSM1/current_state", String, self.robot_cb)
        self.joint_subscriber = rospy.Subscriber("/dvrk/Needle/tip_position_current_pose", Pose, self.tip_pose_cb)
        self.joint_publisher = rospy.Publisher("/dvrk/PSM1/set_position_goal_joint", JointState, queue_size=10)
        self.tipPosDes_publisher = rospy.Publisher("/dvrk/PSM1/tip_position_desired_pose", Pose, queue_size=1)
        self.clikFlag_subscriber = rospy.Subscriber("/dvrk/Needle/clik_flag", Int8, self.clikFlag_cb)
        self.jointIK_subscriber = rospy.Subscriber("/dvrk/PSM1/state_joint_IK", JointState, self.joint_cb)

        #Joint Variable
        self.position = None
        self.euler = None

        #State Variable
        self.start = np.concatenate([prm.startPosition, prm.startEuler])
        self.goal = np.concatenate([prm.goalPosition, prm.goalEuler])
        #self.dist = math.dist(prm.startPosition, prm.goalPosition)
        self.dist = np.absolute(self.goal - self.start)

        
        
        self.goal_pos_state = prm.goalPosTh/(prm.goalPosition - prm.startPosition)
        self.goal_orient_state = prm.goalOrientTh/(prm.goalEuler - prm.startEuler)

        #self.goal_pos_state = tool.divide(prm.goalPosTh, prm.goalPosition - prm.startPosition )
        #self.goal_orient_state = tool.divide(prm.goalOrientTh, prm.goalEuler - prm.startEuler )
        self.start_state = tool.divide(self.goal - self.start, self.dist)

        self.prev_state = np.absolute(self.start_state)
        self.goal_state = np.absolute(np.concatenate([self.goal_pos_state, self.goal_orient_state]))
        self.direction_buffer = tool.abs_buffer(self.start_state)

        self.tip_state = self.start

        self.prev_dist = self.dist
        self.min_dist = self.dist

        self.state = "READY"
        self.save_trajectory = False
        self.temp_reward = 0
        self.best_reward = 0
        self.temp_trajectory = []
        self.trajectory = []
        self.jointIK = []
        self.clikFlag = 0

        self.buffer = buffer.Buffer(prm.buffer_capacity, prm.batch_size)

        self.main_loop()
    
    
    def robot_cb(self, msg):
        self.state = msg.data
    #end_callback

    def tip_pose_cb(self, msg):
        self.position = [msg.position.x, msg.position.y, msg.position.z]
        self.euler = [msg.orientation.x, msg.orientation.y, msg.orientation.z]
        #print("position {}".format(self.position))
        #print("euler {}".format(self.euler))
    #end_callback

    def clikFlag_cb(self, msg):
        self.clikFlag = msg.data
    #end_callback

    def joint_cb (self, msg):
        self.jointIK = msg.position
    #end_callback

    '''
    def state_reward_function (self, state, done=False, info=""):
        statePosition = [state[0], state[1], state[2]]
        self.dist = math.dist(prm.goalPosition, statePosition)

        #reward = 1/self.dist

        if (self.dist<prm.goalTh):
            reward = 5e1
            info = "GOAL REACHED"
            done = True

        elif (self.dist>prm.outTh):
            reward = -1e2
            info = "OUT OF BOUND"
            done = True

        elif (self.dist < self.prev_dist):
            #reward = 1*prm.outTh/(1.3*self.dist)
            reward = 1
            #reward = 7*(-self.dist + prm.outTh)

        else :
            reward = -0.03
            #reward = -self.dist/5
            #reward = -math.exp(-prm.outTh+self.dist)
            

        self.prev_dist = self.dist

        return reward, info, done
    
    #end_state_reward_function

    def joint_reward_function(self, joint, reward=0, done=False, info=""):
        for i in range(6):
            #print(joint)
            #print(prm.q_min)
            if (joint[i]<=prm.q_min[i]):
                info = info + " - joint " + str(i) + " MIN"
                reward += -1e1
                done = True + done

            elif (joint[i]>=prm.q_max[i]):
                info = info + " - joint " + str(i) + " MAX"
                reward += -1e1
                done = True + done

        return reward, info, done

                
    #end_joint_reward_function
    '''

    def reward_function (self, state, joint, reward=0, done=False, info=""):
        statePosition = [state[0], state[1], state[2]]
        temp = math.dist(self.goal_pos_state, statePosition)
        goal_bool = True


         #Joint Section
        for i in range(6):

            if (joint[i]<=prm.q_min[i]):
                info = info + " - joint " + str(i+1) + " MIN"
                reward = -1
                done = True

            elif (joint[i]>=prm.q_max[i]):
                info = info + " - joint " + str(i+1) + " MAX"
                reward = -1
                done = True

        #Out of the workspace
        if (temp>prm.outTh):
            reward = -1
            info = "OUT OF BOUND"
            done = True

        #Find the goal
        for i in range(len(state)):
            goal_bool *=  abs(state[i]) <= self.goal_state[i]
            
        if (goal_bool):
            reward = 1
            info = "GOAL REACHED"
            done = True

        return reward, info, done
    #end_reward_function

    def main_loop(self):
        
        period = 0.05
        f = 1/period
        rate = rospy.Rate(f)

        while (self.state != 'READY'):
            rospy.sleep(period/3)

        #while not rospy.is_shutdown():
        #tool.load_weights(acp.actor_model, acp.critic_model, acp.target_actor, acp.target_critic)
        if self.save_trajectory:
            tool.load_weights(acp.actor_model, acp.critic_model, acp.target_actor, acp.target_critic)
            prm.total_episodes = 5

        for ep in range(prm.total_episodes):

            #print(self.prev_state)


            prev_state = self.prev_state
            prev_tip_state = self.tip_state
            episodic_reward = 0
            action_counter = 0
            reward = 0
            info = ""
            self.temp_trajectory.clear()
            
            while True:
               
                tf_prev_state = tnf.expand_dims(tnf.convert_to_tensor(prev_state), 0)

                action = acp.policy(tf_prev_state, tool.noise_object(tool.ou_noise, ep, info))
                action_counter += 1

                action_ph = np.array(action[0])
                #print("ACTION -> {}".format(action_ph))

                state = np.zeros((prm.num_states))
                tip_state = np.zeros((prm.num_states))
                done = False

                for i in range(prm.num_states):
                    state[i] = prev_state[i] + action_ph[i]
                    tip_state[i] = prev_tip_state[i] - (action_ph[i]*self.dist[i]*self.direction_buffer[i])
                
                
                self.jointIK = []
                #self.jointIK = np.ones((6))*0.2
                
                while ((self.clikFlag == 0) or (self.jointIK == [])):
                    #self.tipPosDes_publisher.publish(tool.convert_to_PoseMsg(state))
                    self.tipPosDes_publisher.publish(tool.convert_to_PoseMsg(tip_state))
                    #print(self.jointIK)
                    #print(self.clikFlag)
                    rospy.sleep(0.05)
                '''    '''

                self.clikFlag = 0
                #print(action_counter)
                #print(state)

                if (action_counter < prm.max_actions):
                    #reward, info, done = self.state_reward_function(state)
                    #reward, info, done = self.joint_reward_function(self.jointIK, reward, done, info)
                    reward, info, done = self.reward_function(state, self.jointIK)

                else:
                    reward = 0
                    info = "MAX ACTION"
                    done = True

                self.buffer.record((prev_state, np.array(action), reward, state))
                episodic_reward += reward
                
                self.buffer.learn()
                acp.update_target(acp.target_actor.variables, acp.actor_model.variables, prm.tau)
                acp.update_target(acp.target_critic.variables, acp.critic_model.variables, prm.tau)

                
                prev_state = state
                prev_tip_state = tip_state

                if (self.save_trajectory):
                    self.temp_trajectory.append(state)
                    
                # End this episode when `done` is True
                if done:
                    break
                
                self.joint_publisher.publish(tool.convert_to_JointStateMsg(self.jointIK))

                rate.sleep()
                

                #self.joint_publisher.publish(pos_msg)
                #print("SERGIO UP")
                #rate.sleep()

            prm.ep_reward_list.append(episodic_reward)
            avg_reward = tool.mean_each_x_episodes(prm.ep_reward_list, 1)
            #print("Episode * {} * Avg Reward is ==> {}".format(ep, avg_reward))
            print("Episode * {} * Ep Reward is ==> {}".format(ep, episodic_reward))
            print("End Tip State: " + tool.print_round_array(prev_tip_state))
            print("End State: " + tool.print_round_array(state))
            print("End Condition: {}".format(info))
            print("Dist: " + tool.print_round_array(self.dist))
            print("Goal Condition: " + tool.print_round_array(self.goal_state))
            print("------------------------------------------------------------")

            if (self.save_trajectory):
                if ((info == "GOAL REACHED") and (episodic_reward > self.best_reward)): 
                    self.trajectory = self.temp_trajectory.copy()
                    self.best_reward = episodic_reward

            prm.avg_reward_list.append(avg_reward)

        if (self.save_trajectory):
            tool.save_on_file(self.trajectory)
            print("SERGIOVED")
        
        tool.plot_average_reward (prm.avg_reward_list)
        tool.save_weights(acp.actor_model, acp.critic_model, acp.target_actor, acp.target_critic)
        

if __name__ == '__main__':
    rospy.init_node('DVRK_DDPG')
    ddpg()
    rospy.spin()