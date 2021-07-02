#ifndef _utilities_hpp
#define _utilities_hpp

#define COMPILE_ROS_HELPER			0 //compile ros libs

#include <cmath>
// Socket
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <sys/time.h>
#include <netdb.h>
#include <Eigen/Dense>


using namespace std;  //calling the standard directory
using namespace Eigen;

namespace utilities{ 

	inline Matrix3d rotx(double alpha){
		Matrix3d Rx;
		Rx << 	1,0,0,
			0,cos(alpha),-sin(alpha),
			0,sin(alpha), cos(alpha);
		return Rx;
	}

	inline Matrix3d roty(double beta){
		Matrix3d Ry;
		Ry << 	cos(beta),0,sin(beta),
			0,1,0,
			-sin(beta),0, cos(beta);
		return Ry;
	}

	inline Matrix3d rotz(double gamma){
		Matrix3d Rz;
		Rz << 	cos(gamma),-sin(gamma),0,
			sin(gamma),cos(gamma),0,
			0,0, 1;
		return Rz;
	}


	inline Matrix4d rotx_T(double alpha){
		Matrix4d Tx = Matrix4d::Identity();
		Tx.block(0,0,3,3) = rotx(alpha);
		return Tx;
	}

	inline Matrix4d roty_T(double beta){
		Matrix4d Ty = Matrix4d::Identity();
		Ty.block(0,0,3,3) = roty(beta);
		return Ty;
	}

	inline Matrix4d rotz_T(double gamma){
		Matrix4d Tz = Matrix4d::Identity();
		Tz.block(0,0,3,3) = rotz(gamma);
		return Tz;
	}

	inline Matrix3d skew(Vector3d v)
	{
		Matrix3d S;
		S << 0,	-v[2],	 v[1],		//Skew-symmetric matrix
			v[2],	    0,	-v[0],
			-v[1],	 v[0], 	   0;
		return S;
	}


	inline Matrix3d L_matrix(Matrix3d R_d, Matrix3d R_e)
	{
		Matrix3d L = -0.5 * (skew(R_d.col(0))*skew(R_e.col(0)) + skew(R_d.col(1))*skew(R_e.col(1)) + skew(R_d.col(2))*skew(R_e.col(2)));
		return L;
	}



    inline Vector3d rotationMatrixError(const Matrix3d& R_d,const Matrix3d& R_e)
	{
        Vector3d e_o;
//        std::cout << "R_d : " << R_d << std::endl;
//        std::cout << "R_e : " << R_e << std::endl;
        e_o << 0.5 * (skew(R_e.col(0))*R_d.col(0) +
                      skew(R_e.col(1))*R_d.col(1) +
                      skew(R_e.col(2))*R_d.col(2));
        Eigen::Quaterniond q_e(R_e);
        Eigen::Quaterniond q_d(R_d);
        Eigen::Quaterniond q = q_d*q_e.inverse();
        e_o << q.x(), q.y(), q.z();
        return e_o;
	}


	inline Vector3d r2quat(Matrix3d R_iniz, double &eta)
	{
		Vector3d epsilon;
		int iu, iv, iw;

		if ( (R_iniz(0,0) >= R_iniz(1,1)) && (R_iniz(0,0) >= R_iniz(2,2)) )
		{
			iu = 0; iv = 1; iw = 2;
		}
		else if ( (R_iniz(1,1) >= R_iniz(0,0)) && (R_iniz(1,1) >= R_iniz(2,2)) )
		{
			iu = 1; iv = 2; iw = 0;
		}
		else
		{
			iu = 2; iv = 0; iw = 1;
		}

		double r = sqrt(1 + R_iniz(iu,iu) - R_iniz(iv,iv) - R_iniz(iw,iw));
		Vector3d q;
		q <<  0,0,0;
		if (r>0)
		{
		double rr = 2*r;
		eta = (R_iniz(iw,iv)-R_iniz(iv,iw)/rr);
		epsilon[iu] = r/2;
		epsilon[iv] = (R_iniz(iu,iv)+R_iniz(iv,iu))/rr;
		epsilon[iw] = (R_iniz(iw,iu)+R_iniz(iu,iw))/rr;
		}
		else
		{
		eta = 1;
		epsilon << 0,0,0;
		}
		return epsilon;
	}


	inline Vector4d rot2quat(Matrix3d R){

		double m00, m01, m02, m10, m11, m12, m20, m21, m22;

		m00 = R(0,0);
		m01 = R(0,1);
		m02 = R(0,2);
		m10 = R(1,0);
		m11 = R(1,1);
		m12 = R(1,2);
		m20 = R(2,0);
		m21 = R(2,1);
		m22 = R(2,2);

		double tr = m00 + m11 + m22;
		double qw, qx, qy, qz, S;
		Vector4d quat;

		if (tr > 0) { 
		  S = sqrt(tr+1.0) * 2; // S=4*qw 
		  qw = 0.25 * S;
		  qx = (m21 - m12) / S;
		  qy = (m02 - m20) / S; 
		  qz = (m10 - m01) / S; 
		} else if ((m00 > m11)&(m00 > m22)) { 
		  S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
		  qw = (m21 - m12) / S;
		  qx = 0.25 * S;
		  qy = (m01 + m10) / S; 
		  qz = (m02 + m20) / S; 
		} else if (m11 > m22) { 
		  S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
		  qw = (m02 - m20) / S;
		  qx = (m01 + m10) / S; 
		  qy = 0.25 * S;
		  qz = (m12 + m21) / S; 
		} else { 
		  S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
		  qw = (m10 - m01) / S;
		  qx = (m02 + m20) / S;
		  qy = (m12 + m21) / S;
		  qz = 0.25 * S;
		}

		quat << qw, qx, qy, qz;
		return quat;

	}



	 //Matrix ortonormalization
	inline MatrixXd matrixOrthonormalization(MatrixXd R){

		SelfAdjointEigenSolver<MatrixXd> es(R.transpose()*R);
		Vector3d D = es.eigenvalues();
		Matrix3d V = es.eigenvectors();
		R = R*((1/sqrt(D(0)))*V.col(0)*V.col(0).transpose() + (1/sqrt(D(1)))*V.col(1)*V.col(1).transpose() + (1/sqrt(D(2)))*V.col(2)*V.col(2).transpose());

		return R;
	}

	//******************************************************************************
	inline Vector3d quaternionError(Matrix4d Tbe, Matrix4d Tbe_d)
	{
		double eta, eta_d;
		Matrix3d R = Tbe.block(0,0,3,3);		//Matrix.slice<RowStart, ColStart, NumRows, NumCols>();	
		Vector3d epsilon = r2quat(R, eta);
		Matrix3d R_d = Tbe_d.block(0,0,3,3);
		Vector3d epsilon_d = r2quat(R_d, eta_d);
		Matrix3d S = skew(epsilon_d);
		Vector3d eo = eta*epsilon_d-eta_d*epsilon-S*epsilon;
		return eo;
	}


	inline Vector3d versorError(Vector3d P_d, Matrix3d Tbe_e, double &theta)
	{
		Vector3d P_e = Tbe_e.block(0,3,3,1);
		Vector3d u_e = Tbe_e.block(0,0,3,1);

		Vector3d u_d = P_d - P_e;
		u_d = u_d/ u_d.norm();
		
		Vector3d r = skew(u_e)*u_d;
		double nr = r.norm();
			
	  if (nr >0){
			r = r/r.norm();
			

			//be carefult to acos( > 1 )
			double u_e_d = u_e.transpose()*u_d;		
			if( fabs(u_e_d) <= 1.0 ) {
				theta = acos(u_e.transpose()*u_d);
			}
			else {
				theta = 0.0;
			}
			
			Vector3d error = r*sin(theta);
			return error;
		}else{
			theta = 0.0;
			return Vector3d::Zero();
		}
	}


	/*Matrix3d utilities::rotation ( float theta, Vector3d r ) {
	     
	    Matrix3d R = Zeros;
	 
	    R[0][0] = r[0]*r[0]*(1-cos(theta)) + cos(theta);
	    R[1][1] = r[1]*r[1]*(1-cos(theta)) + cos(theta);
	    R[2][2] = r[2]*r[2]*(1-cos(theta)) + cos(theta);
	 
	    R[0][1] = r[0]*r[1]*(1-cos(theta)) - r[2]*sin(theta);
	    R[1][0] = r[0]*r[1]*(1-cos(theta)) + r[2]*sin(theta);
	 
	    R[0][2] = r[0]*r[2]*(1-cos(theta)) + r[1]*sin(theta);
	    R[2][0] = r[0]*r[2]*(1-cos(theta)) - r[1]*sin(theta);
	     
	    R[1][2] = r[1]*r[2]*(1-cos(theta)) - r[0]*sin(theta);
	    R[2][1] = r[1]*r[2]*(1-cos(theta)) + r[0]*sin(theta);
	     
	 
	    return R;
	}*/


	inline Matrix3d XYZ2R(Vector3d angles) {
	  	
	  	Matrix3d R = Matrix3d::Zero(); 
	  	Matrix3d R1 = Matrix3d::Zero(); 
	  	Matrix3d R2 = Matrix3d::Zero(); 
	  	Matrix3d R3 = Matrix3d::Zero();

		float cos_phi = cos(angles[0]);
		float sin_phi = sin(angles[0]);
		float cos_theta = cos(angles[1]);
		float sin_theta = sin(angles[1]);
		float cos_psi = cos(angles[2]);
		float sin_psi = sin(angles[2]);

		R1  << 1, 0      , 0, 
			        0, cos_phi, -sin_phi, 
			        0, sin_phi, cos_phi;

		R2  << cos_theta , 0, sin_theta, 
			        0        , 1, 0       , 
			        -sin_theta, 0, cos_theta;

		R3  << cos_psi, -sin_psi, 0, 
			        sin_psi, cos_psi , 0,
			        0      , 0       , 1;

		R = R1*R2*R3;

		return R;
	}

	// This method computes the XYZ Euler angles from the Rotational matrix R.
	inline Vector3d R2XYZ(Matrix3d R) {
		float phi=0.0, theta=0.0, psi=0.0;
		Vector3d XYZ = Vector3d::Zero();
		
		theta = asin(R(0,2));
		
		if(fabsf(cos(theta))>pow(10.0,-10.0))
		{
			phi=atan2(-R(1,2)/cos(theta), R(2,2)/cos(theta));
			psi=atan2(-R(0,1)/cos(theta), R(0,0)/cos(theta));
		}
		else
		{
			if(fabsf(theta-M_PI/2.0)<pow(10.0,-5.0))
			{
				psi = 0.0;
				phi = atan2(R(1,0), R(2,0));
				theta = M_PI/2.0;
			}
			else
			{
				psi = 0.0;
				phi = atan2(-R(1,0), R(2,0));
				theta = -M_PI/2.0;
			}
		}
		
		XYZ << phi,theta,psi;
		return XYZ;
	}

	inline Matrix3d angleAxis2Rot(Vector3d ri, double theta){
	Matrix3d R;
	R << ri[0]*ri[0] * (1 - cos(theta)) + cos(theta)           , ri[0] * ri[1] * (1 - cos(theta)) - ri[2] * sin(theta) , ri[0] * ri[2] * (1 - cos(theta)) + ri[1] * sin(theta),
	         ri[0] * ri[1] * (1 - cos(theta)) + ri[2] * sin(theta) , ri[1]*ri[1] * (1 - cos(theta)) + cos(theta)           , ri[1] * ri[2] * (1 - cos(theta)) - ri[0] * sin(theta),
	         ri[0] * ri[2] * (1 - cos(theta)) - ri[1] * sin(theta) , ri[1] * ri[2] * (1 - cos(theta)) + ri[0] * sin(theta) , ri[2]*ri[2] * (1 - cos(theta)) + cos(theta);

	return R;

	}



	inline Vector3d butt_filter(Vector3d x, Vector3d x1, Vector3d x2, double omega_n, double zita, float ctrl_T){
		//applico un filtro di Butterworth del secondo ordine (sfrutto Eulero all'indietro)
		return x1*(2.0 + 2.0*omega_n*zita*ctrl_T)/(omega_n*omega_n*ctrl_T*ctrl_T + 2.0*omega_n*zita*ctrl_T + 1.0) - x2/(omega_n*omega_n*ctrl_T*ctrl_T + 2.0*omega_n*zita*ctrl_T + 1.0) + x*(omega_n*omega_n*ctrl_T*ctrl_T)/(omega_n*omega_n*ctrl_T*ctrl_T + 2.0*omega_n*zita*ctrl_T + 1.0);
	}



	/// converts a rate in Hz to an integer period in ms.
	inline uint16_t rateToPeriod(const double & rate) {
		if (rate > 0)
			return static_cast<uint16_t> (1000.0 / rate);
		else
			return 0;
	}


	inline void angle2quaternion(const double &roll, const double &pitch, const double &yaw, double *w, double *x,
							 double *y, double *z) {
		double sR2, cR2, sP2, cP2, sY2, cY2;
		sincos(roll * 0.5, &sR2, &cR2);
		sincos(pitch * 0.5, &sP2, &cP2);
		sincos(yaw * 0.5, &sY2, &cY2);

		// TODO: change rotation order
		// this follows pre- 2012 firmware rotation order: Rz*Rx*Ry
		//  *w = cP2 * cR2 * cY2 - sP2 * sR2 * sY2;
		//  *x = cP2 * cY2 * sR2 - cR2 * sP2 * sY2;
		//  *y = cR2 * cY2 * sP2 + cP2 * sR2 * sY2;
		//  *z = cP2 * cR2 * sY2 + cY2 * sP2 * sR2;

		// Rz*Ry*Rx for 2012 firmware on the LL:
		*w = cP2 * cR2 * cY2 + sP2 * sR2 * sY2;
		*x = cP2 * cY2 * sR2 - cR2 * sP2 * sY2;
		*y = cR2 * cY2 * sP2 + cP2 * sR2 * sY2;
		*z = cP2 * cR2 * sY2 - cY2 * sP2 * sR2;
	}
	


	    //Quaternion to rotration Matrix
	inline Matrix3d QuatToMat(Vector4d Quat){
		Matrix3d Rot;
		double s = Quat[0];
		double x = Quat[1];
		double y = Quat[2];
		double z = Quat[3];
		Rot << 1-2*(y*y+z*z),2*(x*y-s*z),2*(x*z+s*y),
		2*(x*y+s*z),1-2*(x*x+z*z),2*(y*z-s*x),
		2*(x*z-s*y),2*(y*z+s*x),1-2*(x*x+y*y);
		return Rot;
	}

	    //Writing
    inline int create_socket(const char* dest, int port, int *sock) {
        struct sockaddr_in sock_in;
		struct hostent *h;
		int error;

        sock_in.sin_family = AF_INET;
        sock_in.sin_port = htons(port);
        h = gethostbyname(dest);

        if (h == 0) {
            //std::cout << red << "Gethostbyname failed"
            //          << strerror(errno) << def << std::endl;
			exit(1);
		}

        bcopy(h->h_addr, &sock_in.sin_addr, h->h_length);
        *sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        error = connect(*sock, (struct sockaddr*) &sock_in, sizeof(sock_in));
        if ( error == -1 ) {
            //std::cout << red << "[helper::create_socket] Error on connect: "
            //          << strerror(errno) << def << std::endl;
        }
        return error;
	}
	
	inline long tic_toc(struct timeval *timer_start, struct timeval *timer_end) {
		long mtime, secs, usecs;    
		
		if(timer_end == NULL) 
			gettimeofday(timer_start, NULL);
		else {
			gettimeofday(timer_end, NULL);
			secs  = timer_end->tv_sec  - timer_start->tv_sec;
			usecs = timer_end->tv_usec - timer_start->tv_usec;
			mtime = ((secs) * 1000 + usecs/1000.0) + 0.5;
			return mtime;
		}
		return 0;
	}

	    //Reading socket
    inline bool listener_socket(int port_number, int *sock, size_t buf_len = 1024) {
        sockaddr_in si_me;
        memset((char *) &si_me, 0, sizeof(si_me));

        if ( (*sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1 ) {
            //std::cout << red << "[helper::create_socket] Error during socket open: "
            //          << strerror(errno) << def << std::endl;
			return false;
		}

        // Set the right buffer
        int rcv_size = buf_len;
        if ( setsockopt(*sock,SOL_SOCKET,SO_RCVBUF,&rcv_size,sizeof(rcv_size)) < 0) {
            std::cout << "[read_socket] Error setting UDP receive buffer" << std::endl;
        }

        // Allow connections to any address port
		si_me.sin_family = AF_INET;
		si_me.sin_port = htons(port_number);
		si_me.sin_addr.s_addr = htonl(INADDR_ANY);
	
        if ( bind(*sock, (struct sockaddr*)&si_me, sizeof(si_me)) == -1 ) {
            //std::cout << red << "[helper::create_socket] Error during bind: "
            //          << strerror(errno) << def << std::endl;
			return false;
		}
		return true;
	}

}


#if COMPILE_ROS_HELPER
namespace utilities_ros {

	//input pos, quaternion
	inline void tf_publish( Vector<3> p, Vector<4> quat, std::string ref_frame, std::string target_frame) {

	    static tf::TransformBroadcaster br;
	    tf::Transform transform;
	    transform.setOrigin( tf::Vector3( p[0], p[1], p[2]) );
	    tf::Quaternion q(quat[1], quat[2],quat[3], quat[0]);
	    transform.setRotation(q);
	    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), ref_frame, target_frame));
	}

	//input pos, R matrix
	inline void tf_publish( Vector<3> p, Matrix<3> R, std::string ref_frame, std::string target_frame) {

	    static tf::TransformBroadcaster br;
	    tf::Transform transform;
	    transform.setOrigin( tf::Vector3( p[0], p[1], p[2]) );
	    Vector<4> quat = helper::MatToQuat( R);
	    tf::Quaternion q(quat[1], quat[2],quat[3], quat[0]);
	    transform.setRotation(q);
	    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), ref_frame, target_frame));
	}
}

#endif


#endif
