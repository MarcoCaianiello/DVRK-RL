// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>
#include <tf2/LinearMath/Quaternion.h>

// CONVERSIONS
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>

// PSM Kinematics
#include <dvrk_rl/PSM_robot.h>

// NEEDLE Kinematics
#include "dvrk_rl/needle.h"

using namespace std;

typedef Matrix<double, Dynamic, 6> PathMatrix;


class TIP_POS{
    public:
        TIP_POS();
        void jointState_cb(sensor_msgs::JointState);
        void tipPosition_cb(geometry_msgs::Pose);
        void path_cb(nav_msgs::Path);
        //void tipPosition_cb(KDL::Frame, KDL::JntArray);
        void execute_path(PathMatrix);
        sensor_msgs::JointState jointToMsg(KDL::JntArray);
        void test ();
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _jointState_sub;
        ros::Publisher _jointIK_pub;
        ros::Publisher _tipPos_pub;
        ros::Publisher _clikFlag_pub;
        ros::Subscriber _tipPath_sub;
        ros::Subscriber _tipPos_sub;
        ros::Publisher _joint_pub;
        PSM_robot psm;
        KDL::JntArray q_min;
        KDL::JntArray q_max;
        KDL::JntArray q_kdl;
        KDL::JntArray q_kdl_prev;
        KDL::JntArray q_kdl_des;
        geometry_msgs::Pose tip_pose;
        bool path_flag = false;
        PathMatrix tip_path;
        int nStep = 0;
        //geometry_msgs::Point point;
        //geometry_msgs::Quaternion quaternion;

};

TIP_POS::TIP_POS () {
    ROS_INFO_STREAM("TIP_POSE STARTED");
    _jointState_sub = _nh.subscribe("/dvrk/PSM1/state_joint_current",0,&TIP_POS::jointState_cb,this);
    _tipPos_pub = _nh.advertise<geometry_msgs::Pose>("/dvrk/Needle/tip_position_current_pose", 0);
    _tipPath_sub = _nh.subscribe("/dvrk/PSM1/tip_desired_path",0,&TIP_POS::path_cb,this);
    _tipPos_sub = _nh.subscribe("/dvrk/PSM1/tip_position_desired_pose",0,&TIP_POS::tipPosition_cb,this);
    _clikFlag_pub = _nh.advertise<std_msgs::Int8>("/dvrk/Needle/clik_flag", 0);
    _jointIK_pub = _nh.advertise<sensor_msgs::JointState>("/dvrk/PSM1/state_joint_IK", 0);
    _joint_pub = _nh.advertise<sensor_msgs::JointState>("/dvrk/PSM1/set_position_goal_joint", 0);
    

    q_kdl.resize(6);
    
    //ros::spin();
}

void TIP_POS::path_cb (nav_msgs::Path msg){
    if (!path_flag){
        path_flag = true;
        nStep = msg.poses.size();
        tip_path.resize(nStep, 6);

        for (int i=0; i<msg.poses.size(); i++){

            tip_path(i,0) = msg.poses[i].pose.position.x;
            tip_path(i,1) = msg.poses[i].pose.position.y;
            tip_path(i,2) = msg.poses[i].pose.position.z;
            tip_path(i,3) = msg.poses[i].pose.orientation.x;
            tip_path(i,4) = msg.poses[i].pose.orientation.y;
            tip_path(i,5) = msg.poses[i].pose.orientation.z;

        }

        execute_path(tip_path);
    }
    
    
}

void TIP_POS::execute_path(PathMatrix path){

    //ROS_INFO_STREAM(q_kdl.data.size());
    //cout << "T_des: " << T_des.p.data[0] << " " << T_des.p.data[1] << " " << T_des.p.data[2] << endl;

    ros::Rate r(1/0.05);
    tf2::Quaternion q;
    KDL::Frame T_des;

    for (int i=0; i<nStep; i++){

        
        q.setRPY(path(i,3), path(i,4), path(i,5));
        q.normalize();

        T_des.p.x(path(i,0));
        T_des.p.y(path(i,1));
        T_des.p.z(path(i,2));

        T_des.M = KDL::Rotation::RPY(path(i,3), path(i,4), path(i,5));


        //q_kdl_des = psm.backwardKinematics(T_des, q_kdl_prev);
        q_kdl_des = psm.backwardKinematicsFromNeedleTip(T_des, q_kdl_prev);

        _joint_pub.publish(jointToMsg(q_kdl_des));

        
        /*
        ROS_INFO_STREAM(q_kdl_des.data[0] << " " <<
                    q_kdl_des.data[1] << " " <<
                    q_kdl_des.data[2] << " " <<
                    q_kdl_des.data[3] << " " <<
                    q_kdl_des.data[4] << " " <<
                    q_kdl_des.data[5] << " ");
        */
        r.sleep();

    }
    double a, b, g;
    

    KDL::Frame tip_pose_kdl = psm.forwardKinematics(q_kdl_des);
    tip_pose_kdl.M.GetRPY(a, b, g);
    cout << "Tip_Pose: " << tip_pose_kdl.p.x() << " " << tip_pose_kdl.p.y() << " " << tip_pose_kdl.p.z() << endl;
    cout << "Tip_Orient: " << a << " " << b << " " << g << endl;
    
}

void TIP_POS::jointState_cb(sensor_msgs::JointState msg){
    //ROS_INFO_STREAM("SERGIO_cb")
    
    double roll, pitch, yaw;


    for (int i=0; i<msg.position.size(); i++){
        q_kdl.data[i] = msg.position[i];
    }

    KDL::Frame tip_pose_kdl = psm.forwardKinematics(q_kdl);
    
    psm.forwardKinematicsNeedleTip(q_kdl);

    //cout << msg.position[0] << " " << msg.position[1] << " " << msg.position[2] << " " << msg.position[3] << " " << msg.position[4] << " " << msg.position[5] << endl;
    //cout << "Tip_Pose: " << tip_pose_kdl.p.x() << " " << tip_pose_kdl.p.y() << " " << tip_pose_kdl.p.z() << endl;

    //ros::Duration(0.5).sleep();

    tf::poseKDLToMsg(tip_pose_kdl, tip_pose);


    tf::Quaternion q(tip_pose.orientation.x, tip_pose.orientation.y, tip_pose.orientation.z, tip_pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    tip_pose.orientation.x = roll;
    tip_pose.orientation.y = pitch;
    tip_pose.orientation.z = yaw;
    tip_pose.orientation.w = 0;

    q_kdl_prev = q_kdl;

    //cout << "Tip_Pose: " << tip_pose_kdl.p.x() << " " << tip_pose_kdl.p.y() << " " << tip_pose_kdl.p.z() <<
    //                        " " << roll << " " << pitch << " " << yaw << endl;


    _tipPos_pub.publish(tip_pose);
    
    //tipPosition_cb(tip_pose_kdl, q_kdl);

    //ros::Duration(0.5).sleep();
    //_jointIK_pub.publish(jointToMsg(q_kdl));

}

/*
void TIP_POS::tipPosition_cb(KDL::Frame T_des, KDL::JntArray q_i){

    //ROS_INFO_STREAM(q_kdl.data.size());
    //cout << "T_des: " << T_des.p.data[0] << " " << T_des.p.data[1] << " " << T_des.p.data[2] << endl;

    q_kdl_des = psm.backwardKinematics(T_des, q_i);

    _jointIK_pub.publish(jointToMsg(q_kdl_des));

    
    
    ROS_INFO_STREAM(q_kdl_des.data[0] << " " <<
                q_kdl_des.data[1] << " " <<
                q_kdl_des.data[2] << " " <<
                q_kdl_des.data[3] << " " <<
                q_kdl_des.data[4] << " " <<
                q_kdl_des.data[5] << " ");

    ROS_INFO_STREAM("SERGIO_CLIK");
    
}
*/

void TIP_POS::tipPosition_cb(geometry_msgs::Pose msg){
    
    /*
    ROS_INFO_STREAM(msg.position.x << " " <<
                    msg.position.y << " " <<
                    msg.position.z << " " <<
                    msg.orientation.x << " " <<
                    msg.orientation.y << " " <<
                    msg.orientation.z << " " <<
                    msg.orientation.w << " ");
    */

    KDL::Frame T_des;
    tf2::Quaternion q;
    q.setRPY(msg.orientation.x, msg.orientation.y, msg.orientation.z);
    q.normalize();
    msg.orientation.x = q[0];
    msg.orientation.y = q[1];
    msg.orientation.z = q[2];
    msg.orientation.w = q[3];

    tf::poseMsgToKDL(msg, T_des);

    

    //ROS_INFO_STREAM(q_kdl.data.size());
    //T_des.p.data[0] = T_des.p.data[0] - 0.001;
    //T_des.p.data[1] = T_des.p.data[1] + 0.0001;
    //T_des.p.data[2] = T_des.p.data[2] + 0.0001;
    //cout << "T_des: " << T_des.p.data[0] << " " << T_des.p.data[1] << " " << T_des.p.data[2] << endl;

    q_kdl_des = psm.backwardKinematics(T_des, q_kdl_prev);

    std_msgs::Int8 flag;
    flag.data = 1;

    _jointIK_pub.publish(jointToMsg(q_kdl_des));
    _clikFlag_pub.publish(flag);
    
    
    ROS_INFO_STREAM(q_kdl_des.data[0] << " " <<
                q_kdl_des.data[1] << " " <<
                q_kdl_des.data[2] << " " <<
                q_kdl_des.data[3] << " " <<
                q_kdl_des.data[4] << " " <<
                q_kdl_des.data[5] << " ");
    /*
    ROS_INFO_STREAM("SERGIO_CLIK");
    */

}


sensor_msgs::JointState TIP_POS::jointToMsg(KDL::JntArray q){

    sensor_msgs::JointState msg;
    
    for (int i=0; i<6; i++){
        msg.position.push_back(q.data[i]);
    }

    return msg;

}

int main( int argc, char** argv) {
    ros::init(argc, argv, "TIP_POS" );
    TIP_POS tip_pos;
    ros::spin();

    return 0;
} 




