#include <iostream>


#include "rotors_control/lee_position_controller.h"
#include "rotors_estimation/kalmen_filter.h"

//#define dt 0.004f
KalmenFilterEstimation::KalmenFilterEstimation(){
	states_.resize(7);
	states_.setZero();
	 P_ << 	1,0,0,0,0,0,0,
	   		0,1,0,0,0,0,0,
	   		0,0,1,0,0,0,0,
	   		0,0,0,1,0,0,0,
	   		0,0,0,0,1,0,0,
	   		0,0,0,0,0,1,0,
	   		0,0,0,0,0,0,1;

	 Q_ <<	0.1,0,0,0,0,0,0,
	   		0,0.1,0,0,0,0,0,
	   		0,0,0.1,0,0,0,0,
	   		0,0,0,0.1,0,0,0,
	   		0,0,0,0,0.1,0,0,
	   		0,0,0,0,0,0.1,0,
	   		0,0,0,0,0,0,0.01;
	
	 R_ <<	0.1,0,0,0,0,0,
			0,0.1,0,0,0,0,
			0,0,0.1,0,0,0,
			0,0,0,0.1,0,0,
			0,0,0,0,0.1,0,
			0,0,0,0,0,0.1;
}

KalmenFilterEstimation::~KalmenFilterEstimation()
{
}


void KalmenFilterEstimation::Predict(double r_pm_x , double r_pm_y , const rotors_control::LeePositionController& UAV,double dt){
	const Eigen::Matrix3d R_W_I = UAV.odometry_.orientation.toRotationMatrix();
	const double force = UAV.moment_thrust_(3);
	if(!states_init()){
		states_.block<3,1>(3,0) = UAV.odometry_.position;
		states_.block<3,1>(3,0) = R_W_I *UAV.odometry_.velocity;
		states_init_flag = true;

	}
	Eigen::Vector3d moment_control_input = UAV.moment_thrust_.block<3, 1>(0, 0);
	Eigen::VectorXd B;
	B.resize(7);
	B.setZero();
	//B.block<3,1>(0,0) = states_.block<3, 1>(3, 0);
	B.block<3,1>(3,0) = -9.8*e3 + R_W_I*(force*e3)/UAV.vehicle_parameters_.mass_;
	B(6) = 0;
	A_.block<3,3>(0,3) = Eigen::MatrixXd::Identity(3,3)*dt;
	states_ = A_*states_ + B*dt ;
	P_ = A_*P_*A_.transpose() + Q_;


}

bool KalmenFilterEstimation::states_init(){
		return states_init_flag;
}
void KalmenFilterEstimation::Correction(double r_pm_x , double r_pm_y , const rotors_control::LeePositionController& UAV){
		const Eigen::Matrix3d R_W_I = UAV.odometry_.orientation.toRotationMatrix();
		const Eigen::Vector3d omega = UAV.odometry_.angular_velocity;  
		Eigen::VectorXd D(6);
		Eigen::Matrix<double,6,7> H;
		Eigen::Matrix<double,6,6> S;
		Eigen::Matrix<double,7,6> K;

		Eigen::VectorXd y_telta(6);
		Eigen::VectorXd measurement(6);
		D.setZero();
		y_telta.setZero();
		measurement.setZero();
		double r_mp_x = -r_pm_x;
		double r_mp_y = -r_pm_y;
		D << 	r_mp_x*R_W_I(0,0) + r_mp_y*R_W_I(0,1),
				r_mp_x*R_W_I(1,0) + r_mp_y*R_W_I(1,1),
				r_mp_x*R_W_I(2,0) + r_mp_y*R_W_I(2,1),
				-R_W_I(0,0)*omega(2)*r_mp_y + R_W_I(0,1)*omega(2)*r_mp_x + R_W_I(0,2)*(-omega(1)*r_mp_x + omega(0)*r_mp_y), 
				-R_W_I(1,0)*omega(2)*r_mp_y + R_W_I(1,1)*omega(2)*r_mp_x + R_W_I(1,2)*(-omega(1)*r_mp_x + omega(0)*r_mp_y), 
				-R_W_I(2,0)*omega(2)*r_mp_y + R_W_I(2,1)*omega(2)*r_mp_x + R_W_I(2,2)*(-omega(1)*r_mp_x + omega(0)*r_mp_y); 
		H << 	1,0,0,0,0,0,R_W_I(0,2),
				0,1,0,0,0,0,R_W_I(1,2),
				0,0,1,0,0,0,R_W_I(2,2),
				0,0,0,1,0,0,R_W_I(0,0)*omega(1)-R_W_I(0,1)*omega(0),
				0,0,0,0,1,0,R_W_I(1,0)*omega(1)-R_W_I(1,1)*omega(0),
				0,0,0,0,0,1,R_W_I(2,0)*omega(1)-R_W_I(2,1)*omega(0);

		measurement.block<3,1>(0,0) = UAV.odometry_.position;	
		measurement.block<3,1>(3,0) = R_W_I *UAV.odometry_.velocity;
		y_telta = measurement - D - H*states_;
		std::cout << "y_telta: "<< y_telta << std::endl;
		S = H*P_*H.transpose() + R_;
		K = P_*H.transpose()*S.inverse();
		states_ = states_ + K*y_telta; 
		P_ = (Eigen::MatrixXd::Identity(7,7)-K*H)*P_;
}
void KalmenFilterEstimation::get_states(Eigen::VectorXd& states){
	states = states_;
}
