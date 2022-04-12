#ifndef KALMEN_FILTER_H
#define KALMEN_FILTER_H

#include <Eigen/Dense>
#include "rotors_control/lee_position_controller.h"

class KalmenFilterEstimation{
public:
	KalmenFilterEstimation();
	~KalmenFilterEstimation();
	void Predict(double r_pm_x , double r_pm_y , const rotors_control::LeePositionController& UAV,double dt);
	void Correction(double r_pm_x , double r_pm_y , const rotors_control::LeePositionController& UAV);
	void get_states(Eigen::VectorXd&);
	bool states_init();
private:
//============ covarience matrix ============
	Eigen::Matrix<double,7,7> P_;

	Eigen::Matrix<double,7,7> Q_;
	
	Eigen::Matrix<double,6,6> R_;
	
//============ update & correction matrix ============
	Eigen::Matrix<double,7,7> A_ = Eigen::MatrixXd::Identity(7,7);

//============ unknown parameters ============
	double r_mp_z_ = 0;
	Eigen::VectorXd states_;

//============ common vector ============
	Eigen::Vector3d e3 = Eigen::Vector3d(0,0,1);
};
//============ flag ============
	bool states_init_flag = false;

#endif //KALMEN_FILTER_H
