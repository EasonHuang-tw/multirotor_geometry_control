/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H


#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <list>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace rotors_control
{

// Default values for the lee position controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);              // 16, 16, 16
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);        // 14.7, 14.7, 14.7
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.15);           // 2, 1.5, 0.035
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.18);  // 0.52, 0.52, 0.025

class LeePositionControllerParameters
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	LeePositionControllerParameters()
		: position_gain_(kDefaultPositionGain),
		  velocity_gain_(kDefaultVelocityGain),
		  attitude_gain_(kDefaultAttitudeGain),
		  angular_rate_gain_(kDefaultAngularRateGain)
	{
		calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
	}

	Eigen::Matrix4Xd allocation_matrix_;
	Eigen::Vector3d position_gain_;
	Eigen::Vector3d velocity_gain_;
	Eigen::Vector3d attitude_gain_;
	Eigen::Vector3d angular_rate_gain_;
	RotorConfiguration rotor_configuration_;
};

class LeePositionController
{
public:
	LeePositionController();
	~LeePositionController();
	void InitializeParameters();
	void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, nav_msgs::Odometry* error);
	void SetOdometry(const EigenOdometry& odometry);
	void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);

	LeePositionControllerParameters controller_parameters_;
	VehicleParameters vehicle_parameters_;
	Eigen::Matrix3d  Last_R_des;
	Eigen::Vector3d  Last_angular_rate_des;
	Eigen::Matrix3d  Inertia_hat;
	Eigen::Matrix4d  I;

	// for moment of inertia estimation
	int index;
	int full;
	int ICL_N;
	Eigen::Vector3d   mat_now;                    // 3x1
	Eigen::Vector3d   theta_diag_hat;             // 3x1
	Eigen::Vector3d   theta_diag_hat_dot;         // 3x1
	Eigen::Vector3d   last_omega;                 // 3x1
	Eigen::Vector3d   last_moment_control_input;  // 3x1
	Eigen::Vector3d   M_integral;                 // 3x1
	Eigen::Vector3d   M_integral_last;            // 3x1
	Eigen::Vector3d   M_bar;                      // 3x1
	Eigen::Vector3d   last_angular_velocity;      // 3x1
	Eigen::Vector3d   mat_sum;                    // 3x1
	Eigen::Matrix3d   Y_diag;                     // 3x3
	Eigen::Matrix3d   Y_diag_cl;                  // 3x3
	Eigen::Matrix3d   Y_diag_cl_integral;         // 3x3
	Eigen::Matrix3d   Y_diag_cl_integral_last;    // 3x3
	Eigen::Matrix3d   y_diag_cl_lower_case;       // 3x3
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>   mat_FIFO;

	// for mass estimation
	Eigen::Matrix3d   Y_m;                        // 3x3
	Eigen::Vector3d   theta_m_hat;                // 3x1
	Eigen::Vector3d   theta_m_hat_dot;            // 3x1

	int index_m;
	int full_m;
	int ICL_N_m;
	double  theta_m_hat_R;                        // 1x1
	double  theta_m_hat_dot_R;                    // 1x1
	double  mat_mass_now;                         // 1x1
	double  mat_mass_sum;                         // 1x1
	Eigen::Vector3d   Y_m_R;                      // 3x1
	Eigen::Vector3d   Y_m_cl_R_dt;                // 3x1
	Eigen::Vector3d   f_integral;                 // 3x1
	Eigen::Vector3d   f_integral_last;            // 3x1
	Eigen::Vector3d   F;                          // 3x1
	Eigen::Vector3d   last_force_control_input;   // 3x1
	Eigen::Vector3d   y_m_cl_integral;            // 3x1
	Eigen::Vector3d   y_m_cl_integral_last;       // 3x1
	Eigen::Vector3d   y_m_cl_lower_case;          // 3x1
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>   mat_mass_FIFO;

	double last_time;
	double now_time;
	std::chrono::time_point<std::chrono::system_clock> start, end;
	double Psi;
	double dt;

	Eigen::Vector3d   angular_rate_error;
	Eigen::Vector3d   angle_error;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	bool initialized_params_;
	bool controller_active_;

	Eigen::Vector3d normalized_attitude_gain_;
	Eigen::Vector3d normalized_angular_rate_gain_;
	Eigen::MatrixX4d angular_acc_to_rotor_velocities_;
	Eigen::MatrixX4d moment_thrust_to_rotor_velocities_;

	Eigen::Vector3d k_R;
	Eigen::Vector3d k_omega;

	mav_msgs::EigenTrajectoryPoint command_trajectory_;
	EigenOdometry odometry_;


	void ComputeDesiredMoment(const Eigen::Vector3d& acceleration, Eigen::Vector3d* moment_control_input);
	void ComputeDesiredForce(Eigen::Vector3d* force_control_input, Eigen::Vector3d* position_error, Eigen::Vector3d* velocity_error);
};
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
