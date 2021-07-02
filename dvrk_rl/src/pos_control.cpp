// Brief: da Vinci Position Control

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

#include <math.h>
#include <cmath>
#include <sstream>
#include <vector>

#include <stdio.h>

#include "dvrk-dynamics/PSM_dynamics_parameters.h"
#include "dvrk-dynamics/PSM_robot.h"

#define M_PI 3.14159265358979323846

// set up joint state variables

std::vector<double> psm_joint_position;
std::vector<double> psm_joint_velocity;
std::vector<double> psm_joint_effort;

Vector3d x_d;

void jointStateCallback(const sensor_msgs::JointState &msg)
{
        psm_joint_position = msg.position;  // outer_yaw_joint    
        psm_joint_velocity = msg.velocity;  // outer_yaw_joint
        psm_joint_effort = msg.effort;  // outer_yaw_joint   
}


void trajStateCallback(const geometry_msgs::PoseStamped& msg)
{
    x_d(0,0) = 0.0;
    x_d(1,0) = 0.0;
    x_d(2,0) = msg.pose.position.z;
}

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-5}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "dvrk_psm_position_control");

    ros::NodeHandle n, nh_private("~");
    int freq = 100;
    ros::Rate rate(freq);  

    // SUBSCRIBER
    ros::Subscriber joint_state_sub = n.subscribe("/dvrk/PSM2/state_joint_current", 1, jointStateCallback);
    ros::Subscriber traj_position_sub = n.subscribe("/dvrk/PSM/trajectory", 1, trajStateCallback);

    //Publisher
    ros::Publisher joint_state_des_pub = n.advertise<sensor_msgs::JointState>("/dvrk/PSM2/set_position_joint", 1);

    PSM_robot psm_dyn;
   
    Vector6d q = Vector6d::Zero();
    Vector7d dq = Vector7d::Zero();
    Vector6d tau = Vector6d::Zero();

    Matrix6d J = Matrix6d::Zero();
    Matrix3d J3 = Matrix3d::Zero();
    Matrix3d J3_inv = Matrix3d::Zero();
    Matrix4d Te = Matrix4d::Zero();
 
    sensor_msgs::JointState joint_state_desired_msg;

    std::vector<double> joint_state_desired_pos(7,0.05);
    Vector7d q_v;

    Vector3d qdot = Vector3d::Zero();
    Vector3d q3 = Vector3d::Zero();

    // ------------ run() --------------------------
    while (ros::ok()) {

        if (psm_joint_position.size() > 0){
            q << psm_joint_position[0],psm_joint_position[1],psm_joint_position[2],psm_joint_position[3],psm_joint_position[4],psm_joint_position[5];
            q3 << psm_joint_position[0],psm_joint_position[1],psm_joint_position[2];
        }

        if (psm_joint_velocity.size() > 0){
            dq << psm_joint_velocity[0],psm_joint_velocity[1],psm_joint_velocity[2],psm_joint_velocity[3],psm_joint_velocity[4],psm_joint_velocity[5],psm_joint_velocity[6];
        }

        if (psm_joint_effort.size() > 0){
            tau << psm_joint_effort[0],psm_joint_effort[1],psm_joint_effort[2],psm_joint_effort[3],psm_joint_effort[4],psm_joint_effort[5];
        }

        Te = psm_dyn.Te(q);
        J = psm_dyn.J(q);
        J3 = (J).block(0,0,3,3);
        J3_inv = pseudoinverse(J3);

        Vector3d x = Te.block(0,3,3,1);
        Vector3d e_p;
        e_p = x_d - x;

        qdot = 50.0*J3_inv*e_p;

        if (qdot.norm() > 0.001)
            q3 = q3 + qdot*0.005;

        q_v << q3[0],q3[1],q3[2], 0.0, 0.0, 0.0, 0.0;

        Eigen::VectorXd::Map(&joint_state_desired_pos[0], q_v.size()) = q_v;
        joint_state_desired_msg.position.clear();
        joint_state_desired_msg.position = joint_state_desired_pos;

        joint_state_des_pub.publish(joint_state_desired_msg);      

        ros::spinOnce();
        rate.sleep();
        
    }

    return 0;
}
