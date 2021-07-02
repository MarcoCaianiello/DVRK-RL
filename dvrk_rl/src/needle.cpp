#include <stdio.h>
#include <math.h>
#include "dvrk_rl/needle.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

Needle::Needle(const double _radius,
               const KDL::Frame _world_T_center)
{
    radius_ = _radius;
    curvature_ = 1.0/_radius;
    T_center_ = _world_T_center;
    center_T_tip_ = KDL::Frame::Identity();
    center_T_tip_.M = KDL::Rotation::EulerZYX(0,0,M_PI_2);
    center_T_tip_.p = KDL::Vector(0,_radius,0); // assuming semi-circular needle
    center_T_grasp_ = KDL::Frame::Identity(); // TODO: correct this
    J_g_.data = Eigen::Matrix<double,6,2>::Zero();
}

double Needle::getRadius() const
{
    return radius_;
}

KDL::Jacobian Needle::getJ_g() const
{
    return J_g_;
}

KDL::Frame Needle::moveTipTo(const KDL::Frame &des_frame, const KDL::Frame &grasp_T_ee)
{
    // calculate end effector given tip desired frame
    KDL::Frame wTee = KDL::Frame::Identity();
    wTee = des_frame*(center_T_tip_.Inverse())*center_T_grasp_*grasp_T_ee;
    T_center_ = wTee*(grasp_T_ee.Inverse())*(center_T_grasp_.Inverse());

//    std::cout << "des_frame: "<< std::endl << des_frame << std::endl;
//    std::cout << "center_T_tip_.Inverse(): "<< std::endl << center_T_tip_.Inverse() << std::endl;
//    std::cout << "center_T_grasp_: "<< std::endl << center_T_grasp_ << std::endl;
//    std::cout << "offset: "<< std::endl << offset << std::endl;
    return wTee;
}

KDL::Frame Needle::grasp(const double &s, const double &alpha, const KDL::Frame &grasp_T_ee)
{
    KDL::Frame ee_T_n = KDL::Frame::Identity();

    // calculate grasp position
    center_T_grasp_.p = KDL::Vector(0,-radius_*std::cos(s*M_PI),-radius_*std::sin(s*M_PI));
    Eigen::Vector3d p;
    p << center_T_grasp_.p.data[0], center_T_grasp_.p.data[1], center_T_grasp_.p.data[2];
    //std::cout << "p: " << p.transpose() << std::endl;

    // calculate grasp rotation
    KDL::Vector x (1.0,0.0,0.0);
    KDL::Vector y (0,std::sin(s*M_PI),-std::cos(s*M_PI));
    Eigen::Vector3d x_eigen(x.data);
    Eigen::Vector3d y_eigen(y.data);
    Eigen::Vector3d z_eigen = x_eigen.cross(y_eigen);
    KDL::Vector z (z_eigen.x(),z_eigen.y(),z_eigen.z());
    center_T_grasp_.M = KDL::Rotation(x,y,z);
    center_T_grasp_.M.DoRotY(alpha);
    ee_T_n = (center_T_grasp_*grasp_T_ee).Inverse();

    // calculate grasp jacobian (n_twist_g)
    J_g_.data.col(1) << 0.0,0.0,0.0, y_eigen;
    J_g_.data.col(0) << M_PI*radius_*y_eigen, M_PI*y_eigen.cross(-p)/radius_;

    // transform grasp jacobian (n_twist_e)
    Eigen::Matrix<double,6,6> Adne = Eigen::Matrix<double,6,6>::Identity();
    Eigen::Vector3d p_ne;
    tf::vectorKDLToEigen(grasp_T_ee.p,p_ne);
    Adne.block(0,3,3,3) = -utilities::skew(p_ne);
    J_g_.data = Adne*J_g_.data;
    // std::cout << "p_ne: " << p_ne.transpose() << std::endl;
    // std::cout << J_g_.data.transpose() << std::endl;

    return ee_T_n;
}

KDL::Frame Needle::get_world_T_center() const
{
    return T_center_;
}

KDL::Frame Needle::get_center_T_tip() const
{
    return center_T_tip_;
}

KDL::Frame Needle::get_center_T_grasp() const
{
    return center_T_grasp_;
}

