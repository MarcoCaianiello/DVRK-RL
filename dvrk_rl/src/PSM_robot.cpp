/*This file is part of dvrk-dynamics package.
 * Copyright (C) 2017, Giuseppe Andrea Fontanelli
 
 * Email id : giuseppeandrea.fontanelli@unina.it
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
* This code will subscriber integer values from demo_topic_publisher
*/

#include <stdio.h>
#include <math.h>
#include "dvrk_rl/PSM_robot.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif


//******************************************************************************
PSM_robot::PSM_robot(const KDL::JntArray &_q_min,
                     const KDL::JntArray &_q_max)
{
    q_min_ = new KDL::JntArray(_q_min);
    q_max_ = new KDL::JntArray(_q_max);
}

PSM_robot::PSM_robot(){

    Vector6d q_min_eigen;
    Vector6d q_max_eigen;

    q_min_eigen << -M_PI/3, -M_PI/2, 0.05, -M_PI, -M_PI/2, -M_PI/2;
    q_max_eigen << M_PI/3, M_PI/2, 0.25, M_PI, M_PI/2, M_PI/2;

    KDL::JntArray temp(6);

    temp.data << q_min_eigen;
    q_min_ = new KDL::JntArray(temp);

    temp.data << q_max_eigen;
    q_max_ = new KDL::JntArray(temp);

}
//******************************************************************************

//Forward Kinematics Matrix Te
KDL::Frame PSM_robot::forwardKinematics(const KDL::JntArray &q){
    KDL::Frame A0;

    double q1 = q.data[0];
    double q2 = q.data[1];
    double q3 = q.data[2];
    double q4 = q.data[3];
    double q5 = q.data[4];
    double q6 = q.data[5];

    double qs5 = 0.0;
    double qs6 = 0.0;

    double t2 = cos(qs5);
    double t3 = sin(q2);
    double t4 = cos(q1);
    double t5 = sin(qs5);
    double t6 = t4*t5;
    double t7 = cos(qs6);
    double t8 = sin(q1);
    double t13 = t2*t7*t8;
    double t9 = t6-t13;
    double t10 = cos(q2);
    double t11 = sin(qs6);
    double t12 = sin(q4);
    double t14 = t3*t9;
    double t15 = t2*t10*t11;
    double t16 = t14+t15;
    double t17 = cos(q4);
    double t18 = t5*t8;
    double t19 = t2*t4*t7;
    double t20 = t18+t19;
    double t21 = sin(q5);
    double t22 = t9*t10;
    double t27 = t2*t3*t11;
    double t23 = t22-t27;
    double t24 = cos(q5);
    double t25 = t12*t20;
    double t26 = cos(q6);
    double t28 = t23*t24;
    double t35 = t16*t17;
    double t36 = t25-t35;
    double t29 = t21*t36;
    double t30 = t28+t29;
    double t31 = sin(q6);
    double t32 = t12*t16;
    double t33 = t17*t20;
    double t34 = t32+t33;
    double t37 = t7*t10;
    double t38 = t3*t8*t11;
    double t39 = t37+t38;
    double t40 = t17*t39;
    double t41 = t4*t11*t12;
    double t42 = t40+t41;
    double t43 = t3*t7;
    double t48 = t8*t10*t11;
    double t44 = t43-t48;
    double t45 = t12*t39;
    double t46 = t45-t4*t11*t17;
    double t47 = t21*t42;
    double t49 = t24*t44;
    double t50 = t47+t49;
    double t51 = q3-1.56E-2;
    double t52 = t2*t4;
    double t53 = t5*t7*t8;
    double t54 = t52+t53;
    double t55 = t3*t54;
    double t62 = t5*t10*t11;
    double t56 = t55-t62;
    double t57 = t2*t8;
    double t63 = t4*t5*t7;
    double t58 = t57-t63;
    double t59 = t10*t54;
    double t60 = t3*t5*t11;
    double t61 = t59+t60;
    double t64 = t12*t58;
    double t65 = t24*t61;
    double t71 = t17*t56;
    double t72 = t64-t71;
    double t66 = t21*t72;
    double t67 = t65+t66;
    double t68 = t12*t56;
    double t69 = t17*t58;
    double t70 = t68+t69;

    A0.M(0,0) = -t26*t34-t30*t31;
    A0.M(0,1) = -t21*t23+t24*(t25-t16*t17);
    A0.M(0,2) = -t26*t30+t31*t34;
    A0.p(0) = t23*t24*(-9.1E-3)-t21*t36*9.1E-3-t23*t51;
    A0.M(1,0) = t26*t46-t31*t50;
    A0.M(1,1) = -t21*t44+t24*t42;
    A0.M(1,2) = -t26*t50-t31*t46;
    A0.p(1) = t21*t42*(-9.1E-3)-t24*t44*9.1E-3-t44*t51;
    A0.M(2,0) = -t26*t70-t31*t67;
    A0.M(2,1) = -t21*t61+t24*(t64-t17*t56);
    A0.M(2,2) = -t26*t67+t31*t70;
    A0.p(2) = t24*t61*(-9.1E-3)-t21*t72*9.1E-3-t51*t61;
    return A0;
}

//Forward Kinematics Needle Tip
KDL::Frame PSM_robot::forwardKinematicsNeedleTip(const KDL::JntArray &q){
    KDL::Frame A0;
    Vector4d pos_tip;
    KDL::Rotation rot_tip;
    Matrix4d At = getMatrixTipTe();
    KDL::Frame Te(forwardKinematics(q));
    Vector4d pos_Te;
    pos_Te << Te.p(0), Te.p(1), Te.p(2), 1;
    
    pos_tip = At*pos_Te;

    Matrix3d blocco = At.block(0,0,3,3);
    KDL::Rotation temp (blocco(0,0), blocco(0,1), blocco(0,2),
                        blocco(1,0), blocco(1,1), blocco(1,2),
                        blocco(2,0), blocco(2,1), blocco(2,2));
    rot_tip = temp*Te.M;

    //cout << "EE_pose: \n" << pos_Te << endl;
    //cout << "Tip_pose: \n" << pos_tip << endl;

    //cout << "EE_rot: \n" << Te.M << endl;
    //cout << "Tip_rot: \n" << rot_tip << endl;

    A0.p.x(pos_tip[0]);
    A0.p.y(pos_tip[1]);
    A0.p.z(pos_tip[2]);

    A0.M = rot_tip;

    return A0;
}

//Matrix Transformation between needle tip and end-effector
Matrix4d PSM_robot::getMatrixTipTe(){
    Matrix4d A0 = Matrix4d::Zero();

    
/*
    A0(0,0) = -0.1410;
    A0(0,1) = -0.2981;
    A0(0,2) = -0.9441;
    A0(0,3) = -0.0071;

    A0(1,0) = -0.6597;
    A0(1,1) = -0.6827;
    A0(1,2) = 0.3141;
    A0(1,3) = -0.0101;

    A0(2,0) = -0.7382;
    A0(2,1) = 0.6671;
    A0(2,2) = -0.1004;
    A0(2,3) = -0.0194;
*/ 

    A0(0,0) = 1;
    A0(0,3) = -0.0066530704498291;

    A0(1,1) = 1;
    A0(1,3) = -0.022017598152161;

    A0(2,2) = 1;
    A0(2,3) = -0.0033704042434692;
 
    A0(3,3) = 1;

    return A0;
}


//Jacobian matrix J
KDL::Jacobian PSM_robot::jacobianMatrix(const KDL::JntArray &q){

    KDL::Jacobian A0(6);
    A0.data.setZero();

    double q1 = q.data[0];
    double q2 = q.data[1];
    double q3 = q.data[2];
    double q4 = q.data[3];
    double q5 = q.data[4];
    double q6 = q.data[5];

    double qs5 = 0.0;
    double qs6 = 0.0;

    double t2 = cos(qs6);
    double t3 = sin(qs5);
    double t4 = cos(q2);
    double t5 = cos(q1);
    double t6 = cos(qs5);
    double t7 = t5*t6;
    double t8 = sin(q1);
    double t9 = t2*t3*t8;
    double t10 = t7+t9;
    double t11 = t4*t10;
    double t12 = sin(q2);
    double t13 = sin(qs6);
    double t14 = t3*t12*t13;
    double t15 = t11+t14;
    double t16 = sin(q5);
    double t17 = cos(q4);
    double t18 = sin(q4);
    double t19 = q3-1.56E-2;
    double t20 = cos(q5);
    double t21 = t2*t12;
    double t32 = t4*t8*t13;
    double t22 = t21-t32;
    double t23 = t6*t8;
    double t40 = t2*t3*t5;
    double t24 = t23-t40;
    double t25 = t2*t4;
    double t26 = t8*t12*t13;
    double t27 = t25+t26;
    double t28 = t17*t27;
    double t29 = t5*t13*t18;
    double t30 = t28+t29;
    double t31 = t16*t30*9.1E-3;
    double t33 = t19*t22;
    double t34 = t20*t22*9.1E-3;
    double t35 = t31+t33+t34;
    double t36 = t15*t19;
    double t37 = t15*t20*9.1E-3;
    double t38 = t10*t12;
    double t44 = t3*t4*t13;
    double t39 = t38-t44;
    double t41 = t18*t24;
    double t45 = t17*t39;
    double t75 = t41-t45;
    double t42 = t16*t75*9.1E-3;
    double t43 = t36+t37+t42;
    double t46 = t37+t42;
    double t47 = t31+t34;
    double t48 = t3*t5;
    double t52 = t2*t6*t8;
    double t49 = t48-t52;
    double t50 = t6*t12*t13;
    double t53 = t4*t49;
    double t51 = t50-t53;
    double t54 = t3*t8;
    double t55 = t2*t5*t6;
    double t56 = t54+t55;
    double t57 = t19*t51;
    double t58 = t20*t51*9.1E-3;
    double t59 = t12*t49;
    double t60 = t4*t6*t13;
    double t61 = t59+t60;
    double t62 = t18*t56;
    double t63 = t18*t27;
    double t71 = t5*t13*t17;
    double t64 = t63-t71;
    double t69 = t17*t61;
    double t65 = t62-t69;
    double t66 = t58-t16*t65*9.1E-3;
    double t67 = t18*t61;
    double t68 = t17*t56;
    double t70 = -t21+t32;
    double t72 = -t11-t14;
    double t73 = t18*t39;
    double t74 = t17*t24;
    A0(0,0) = t2*t43-t3*t13*t35;
    A0(0,1) = -t24*t35+t5*t13*t43;
    A0(0,2) = t51;
    A0(0,3) = -t15*t47+t22*t46;
    A0(0,4) = -t47*(t73+t74)-t46*t64;
    A0(1,0) = t13*(t4*t5*-1.56E2+q3*t4*t5*1.0E4+t4*t5*t20*9.1E1+t8*t16*t18*9.1E1-t5*t12*t16*t17*9.1E1)*1.0E-4;
    A0(1,1) = t2*t4*1.56E-2-q3*t2*t4-t2*t4*t20*9.1E-3+t8*t12*t13*1.56E-2-q3*t8*t12*t13+t2*t12*t16*t17*9.1E-3-t8*t12*t13*t20*9.1E-3-t4*t8*t13*t16*t17*9.1E-3;
    A0(1,2) = t70;
    A0(1,3) = t16*(t2*t4*t18-t5*t13*t17+t8*t12*t13*t18)*9.1E-3;
    A0(1,4) = t2*t12*t16*9.1E-3-t4*t8*t13*t16*9.1E-3-t2*t4*t17*t20*9.1E-3-t5*t13*t18*t20*9.1E-3-t8*t12*t13*t17*t20*9.1E-3;
    A0(2,0) = t2*(t57+t58-t16*t65*9.1E-3)-t6*t13*t35;
    A0(2,1) = t35*t56+t5*t13*(t57+t58-t16*t65*9.1E-3);
    A0(2,2) = t72;
    A0(2,3) = t22*t66-t47*t51;
    A0(2,4) = t47*(t67+t68)-t64*t66;
    A0(3,0) = t6*t13;
    A0(3,1) = -t54-t55;
    A0(3,3) = t51;
    A0(3,4) = -t67-t68;
    A0(3,5) = -t16*t51-t20*t65;
    A0(4,0) = -t2;
    A0(4,1) = -t5*t13;
    A0(4,3) = t70;
    A0(4,4) = t64;
    A0(4,5) = t16*t22-t20*t30;
    A0(5,0) = -t3*t13;
    A0(5,1) = -t23+t40;
    A0(5,3) = t72;
    A0(5,4) = -t73-t74;
    A0(5,5) = t15*t16-t20*t75;

    return A0;
}


// Inverse Kinematics From Needle Tip
KDL::JntArray PSM_robot::backwardKinematicsFromNeedleTip(const KDL::Frame& T_d, const KDL::JntArray& q_i){

    KDL::Frame Te;
    
    Matrix4d At = getMatrixTipTe();
    KDL::Rotation rot_tip ( At(0,0), At(0,1), At(0,2),
                            At(1,0), At(1,1), At(1,2),
                            At(2,0), At(2,1), At(2,2) );
    Vector4d pos_tip;
    pos_tip << T_d.p.x(), T_d.p.y(), T_d.p.z(), 1;

    Vector4d temp = At.inverse()*pos_tip;
    Te.p.x(temp[0]); 
    Te.p.y(temp[1]);
    Te.p.z(temp[2]); 
    Te.M = rot_tip.Inverse()*T_d.M;

    //cout << "EE_pose: \n" << pos_Te << endl;
    //cout << "Tip_pose: \n" << pos_tip << endl;
    
    double a, b, g;
    Te.M.GetRPY(a, b, g);
    //cout << "EE_rot: \n" << a << " " << b << " " << g << endl;
    //cout << "Tip_rot: \n" << rot_tip << endl;

    return backwardKinematics(Te, q_i);

}


// Inverse Kinematics
KDL::JntArray PSM_robot::backwardKinematics(const KDL::Frame& T_d,
                                            const KDL::JntArray& q_i)
{
    // std::cout << "calculating inverse kinematics... " << std::endl;
    double _Tsam = 0.001; //0.001;        // sampling time
    double _max_pError = 0.001; //0.0001;  // max norm position error
    double _max_oError = 0.05; //0.0005;  // max norm orientation error
    double _Kp = 1000; //1000;           // position gain
    double _Ko = 1000; //500;            // orientation gain
    int _n_max_iter = 300; //100;       // maximum iteration number
    
    Matrix3d Kp = _Kp*Matrix3d::Identity();
    Matrix3d Ko = _Ko*Matrix3d::Identity();
    
    Vector6d q;
    q << q_i.data;
    KDL::JntArray q_kdl(6);
    q_kdl.data << q;
    Vector6d dq = Vector6d::Zero();
    Vector6d dp = Vector6d::Zero();
    Matrix6d W_inv = Matrix6d::Identity();

    KDL::Frame T_e = forwardKinematics(q_kdl);
    KDL::Jacobian J = jacobianMatrix(q_kdl);

    Matrix6d pinvJ = Matrix6d::Zero();
    Matrix6d I = Matrix6d::Identity();

    Matrix6d J_T = Matrix6d::Zero();
    Matrix3d L = Matrix3d::Zero();
    Matrix3d pinvL = Matrix3d::Zero();
    Vector3d vp = Vector3d::Zero();
    Vector3d vo = Vector3d::Zero();
    Vector6d v = Vector6d::Zero();

    int n_iter = 0;

    Vector3d p_d(T_d.p.data);
    Vector3d p_e(T_e.p.data);
    Vector3d ep;
    ep = p_d - p_e;

    

    Eigen::Matrix<double,3,3,RowMajor> Rd(T_d.M.data);
    Eigen::Matrix<double,3,3,RowMajor> Re(T_e.M.data);
    Vector3d eo;
    eo = utilities::rotationMatrixError(Rd, Re);
    double pError = ep.norm();                                     //L_2 norm of vector
    double oError = eo.norm();
    while(n_iter<=_n_max_iter && (pError>_max_pError || oError>_max_oError)){

        //L = utilities::L_matrix(Td.block(0,0,3,3), Te.block(0,0,3,3));
        //FullPivLU<Matrix3d> Core_L(L);
        //pinvL = Core_L.inverse();

        //vp = dp.block(0,0,3,1) + Kp*ep;
        //cout<<"ep: "<<ep<<endl;
        vp =  Kp*ep;
        for (int i=0; i<3; i++){vp[i] += 0.0001;}

        //vo = pinvL*(L.transpose()*dp.block(3,0,3,1) + Ko*eo);
        vo = Ko*eo;
        for (int i=0; i<3; i++){vo[i] += 0.0001;}
        v << vp[0], vp[1], vp[2], vo[0], vo[1], vo[2];
        // std::cout << vo << std::endl;
        //J_T = J.transpose();

        // If we need a weighted pinv
        //FullPivLU<Matrix6d> Core_J(J*W_inv*J_T);
        //pinvJ = W_inv*J_T*Core_J.inverse();
        //FullPivLU<Matrix6d> Core_J(J);

        J = jacobianMatrix(q_kdl);
        const Matrix<double,6,6> j_eigen = J.data.block(0,0,6,6);
        FullPivLU<Matrix6d> lu(j_eigen);
        lu.setThreshold(1e-5);
        if (lu.rank() < 6)
        {
            std::cout << "[WARNING]: jacobian rank < 6!" << std::endl;
            break;
        }

        //pinvJ = J.data.inverse();
        pinvJ = J.data.transpose()*(J.data*J.data.transpose()+0.00001*Eigen::Matrix<double,6,6>::Identity()).inverse();
        //pinvJ = J.data.transpose()*(J.data*J.data.transpose() + 0.001*I).inverse();


        dq = pinvJ*v;
        q += dq*_Tsam;

        /*
        // std::cout << q.transpose() << std::endl;
        for(unsigned int i = 0; i < 6; i++)
        {
            if (q[i] < -M_PI)
            {
                //std::cout << "[WARNING] joint " << i << " < - PI \n";
                q[i] = M_PI + (q[i] + M_PI);
            }

            if (q[i] > M_PI)
            {
                //std::cout << "[WARNING] joint " << i << " > PI \n";
                q[i] = - M_PI + (q[i] - M_PI);
            }
        }
        */
        for(unsigned int i = 0; i < 6; i++)
        {
            if (q[i] < q_min_->data[i]+0.001)
            {
                //std::cout << "[WARNING]: joint " << i+1 << " < lower limit. Value = "<< q(i,0) << "\n";
                q[i] = q_min_->data[i]+0.01;
            }
            if (q[i] > q_max_->data[i]-0.001)
            {
                //std::cout << "[WARNING]: joint " << i+1 << " > upper limit. Value = "<< q(i,0) << "\n";
                q[i] = q_max_->data[i]-0.01;
            }
        }
        

        q_kdl.data << q;

        T_e = forwardKinematics(q_kdl);

        p_e = Vector3d(T_e.p.data);
        Re = Eigen::Matrix<double,3,3,RowMajor>(T_e.M.data);

        ep = p_d - p_e;

        eo = utilities::rotationMatrixError(Rd, Re);

        pError = ep.norm(); //L_2 norm of vector
        oError = eo.norm();
        n_iter++;

//        std::cout << "q : " << q.transpose() << std::endl;
//        std::cout << "q_kdl : " << q_kdl.data.transpose() << std::endl;
//        std::cout << "T_d : " << T_d << std::endl;
//        std::cout << "T_e : " << T_e << std::endl;
//        std::cout << "J : " << J.data << std::endl;
//        std::cout << "R_d : " << Rd << std::endl;
//        std::cout << "R_e : " << Re << std::endl;
//        std::cout << "eo : " << eo << std::endl;
//        std::cout << "pError: " << pError << std::endl;
//        std::cout << "oError : " << oError << std::endl;
//        int c=getchar();

    }

//    std::cout << "q : " << q_kdl.data.transpose() << std::endl;
//    std::cout << "Re : " << Re << std::endl;
//    std::cout << "Rd : " << Rd << std::endl;
//    std::cout << "# iter: " << n_iter << std::endl;
//    cout << "position error: " << pError << std::endl
//         << "orientation Error: " << oError << std::endl;

    if (n_iter >= _n_max_iter)
    {
        std::cout << "[WARNING]: Ik maximum iterations reached." << std::endl;
        cout << "position error: " << pError << std::endl
             << "orientation Error: " << oError << std::endl;
    }
    q_kdl.data << q;
    return q_kdl;
}

void PSM_robot::getJointLimits(KDL::JntArray& q_max,
                               KDL::JntArray& q_min)
{
    q_max.data = this->q_max_->data;
    q_min.data = this->q_min_->data;
}




