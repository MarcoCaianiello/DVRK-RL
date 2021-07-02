// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

// CONVERSIONS
#include <eigen_conversions/eigen_kdl.h>

// PSM Kinematics
#include <dvrk_rl/PSM_robot.h>
// NEEDLE Kinematics
#include "dvrk_rl/needle.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "test_kinematics");
    ros::NodeHandle n;

    // Choose the frequency of the loop
    ros::Rate loop_rate = ros::Rate(10);
    ROS_INFO("test_kinematics node started");

    // Create Robot
    PSM_robot psm;

    std::vector<Vector6d> q_std_eigen;
    Vector6d q1;
    q1 << -1.57,0,0,0,0,0;
    q_std_eigen.push_back(q1);

    while(ros::ok())
    {
        for(unsigned int k = 0; k<q_std_eigen.size(); k++)
        {
            KDL::JntArray q_kdl(6);
            q_kdl.data = q_std_eigen[k];
            KDL::Frame t = psm.forwardKinematics(q_kdl);
            KDL::Jacobian j = psm.jacobianMatrix(q_kdl);

            std::cout << "--------------------" << std::endl;
            std::cout << "q:" << std::endl;
            std::cout << q_kdl.data.transpose() << std::endl;
            std::cout << "forward kinematics:" << std::endl;
            std::cout << t << std::endl;
            std::cout << "jacobian:" << std::endl;
            std::cout << j.data << std::endl;
            std::cout << "--------------------" << std::endl;
        }

        loop_rate.sleep();
        ros::spinOnce();
        return 0;
    }
    return 0;
}
