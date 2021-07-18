#ifndef _NEEDLE_H
#define _NEEDLE_H

#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <fstream>
#include <stdio.h>
#include <math.h>

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>

#include <Eigen/Dense>
#include "utilities.hpp"
#include "eigen_conversions/eigen_kdl.h"

using namespace Eigen;
using namespace std;

class Needle
{
public:

    Needle(const double radius,
           const KDL::Frame _world_T_center);

    KDL::Frame grasp(const double &s,
                     const double &alpha,
                     const KDL::Frame &grasp_T_ee);
    KDL::Frame moveTipTo(const KDL::Frame &des_frame,
                         const KDL::Frame &grasp_T_ee);

    KDL::Frame get_world_T_center() const;
    KDL::Frame get_center_T_grasp() const;
    KDL::Frame get_center_T_tip() const;
    double getRadius() const;

    KDL::Jacobian getJ_g() const;

private:

    double radius_;
    double curvature_;

    KDL::Frame T_center_;
    KDL::Jacobian J_g_;
    KDL::Frame center_T_tip_;
    KDL::Frame center_T_grasp_;
};

#endif // _NEEDLE_H
