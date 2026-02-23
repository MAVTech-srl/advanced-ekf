#pragma once

#include "IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp"

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2; 

namespace UAVmodel
{
	const double mass 				= 2.4344;																	// UAV mass in kilos
	const Eigen::Matrix3d inv_inertia = Eigen::Vector3d(1 / (0.021667 + 0.00034),
														1 / (0.021667 + 0.00034),
														1 / (0.04 + 0.00022)).asDiagonal();						// Inverse of inertia matrix
	const double arm_dist_x 			= 0.174;																// Rotor distance from center of UAV in meters (along x axis)
	const double arm_dist_y 			= 0.174;																// Rotor distance from center of UAV in meters (along y axis)
	const int PROCESS_NOISE_SIZE		= 12;
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> skew_symm(Eigen::Vector<T, 3> in)
{
	Eigen::Matrix<T, 3, 3> out;
	out << 	0.0, -in(2), in(1),
			in(2), 0.0, -in(0),
			-in(1), in(0), 0.0;
	return out;
}

struct Measurements
{
	Eigen::Vector3d px4_position, px4_velocity;
    Eigen::Matrix3d px4_position_cov, px4_velocity_cov, px4_orientation_cov, imu_acc_cov, imu_gyr_cov;
    Eigen::Quaterniond px4_orientation;
    Eigen::Vector3d imu_acc, imu_gyr;
};

MTK_BUILD_MANIFOLD(state_ikfom,
((vect3, pos))
((SO3, rot))
((SO3, offset_R_L_I))
((vect3, offset_T_L_I))
((vect3, vel))
((vect3, bg))
((vect3, ba))
((S2, grav))
((SO3, rot_gps_imu))
((vect3, pos_gps_imu))
((vect3, omega))
((vect3, btau))
);

MTK_BUILD_MANIFOLD(input_ikfom,
((vect3, force))
((vect3, torque))
);

MTK_BUILD_MANIFOLD(process_noise_ikfom,
((vect3, nbg))
((vect3, nba))
((vect3, na_disturb))			// Disturbance noise taking care of aerodynamic and wind effects
((vect3, nbtau))				// Torque bias noise
);

MTK::get_cov<process_noise_ikfom>::type process_noise_cov()
{
	MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
	MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::nbg, 0.00001); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
	MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::nba, 0.00001);   //0.001 0.05 0.0001/out 0.01
	MTK::setDiagonal<process_noise_ikfom, vect3, 6>(cov, &process_noise_ikfom::na_disturb, 0.001);   //0.001 0.05 0.0001/out 0.01
	MTK::setDiagonal<process_noise_ikfom, vect3, 9>(cov, &process_noise_ikfom::nbtau, 0.00001);   //0.001 0.05 0.0001/out 0.01
	return cov;
}

// Eigen::Matrix<double, 33, 1> f(state_ikfom &state, const input_ikfom &input)
// {
// 	Eigen::VectorXd out = Eigen::Matrix<double, 33, 1>::Zero();
// 	out.head(3) 	   = state.vel;
// 	out.segment(3, 3)  = state.omega;
// 	out.segment(6, 3)  = Eigen::Vector3d::Zero();
// 	out.segment(9, 3)  = Eigen::Vector3d::Zero();
// 	out.segment(12, 3) = /*state.grav.get_vect() + */state.rot.toRotationMatrix() * input.force / UAVmodel::mass;
// 	out.segment(15, 3) = Eigen::Vector3d::Zero();		// No state contribution, only noise contribution (i.e. nbg)
// 	out.segment(18, 3) = Eigen::Vector3d::Zero();		// No state contribution, only noise contribution (i.e. nba)
// 	out.segment(21, 3) = Eigen::Vector3d::Zero();		// Gravity
// 	out.segment(24, 3) = Eigen::Vector3d::Zero();		// R_odom_imu
// 	out.segment(27, 3) = Eigen::Vector3d::Zero();		// p_odom_imu
// 	out.segment(30, 3) = -UAVmodel::inv_inertia * ( skew_symm(state.omega) * UAVmodel::inv_inertia * state.omega ) + UAVmodel::inv_inertia * input.torque;
// 	return out;
// }

Eigen::Matrix<double, 36, 35> df_dx(state_ikfom &state, const input_ikfom &input)
{
	// Eigen::Matrix<double, 30, 29> cov = Eigen::Matrix<double, 30, 29>::Zero();
	// cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
	// vect3 acc_;
	// in.acc.boxminus(acc_, s.ba);
	// vect3 omega;
	// in.gyro.boxminus(omega, s.bg);
	// cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix()*MTK::hat(acc_);
	// cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();
	// Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	// Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	// s.S2_Mx(grav_matrix, vec, 21);
	// cov.template block<3, 2>(12, 21) =  grav_matrix; 
	// return cov;
	Eigen::MatrixXd out(36, 35);
	out.setZero();
	out.block<3, 3>(0, 12) << Eigen::Matrix3d::Identity();													// wrt vel
	out.block<3, 3>(12, 3) << -state.rot.toRotationMatrix() * skew_symm(input.force) / UAVmodel::mass;
	// Eigen::Matrix<double, 3, 2> grav_matrix;
	// Eigen::Matrix<double, 2, 1> vec = Eigen::Matrix<double, 2, 1>::Zero();
	// state.S2_Mx(grav_matrix, vec, 21/* state index (apparently) */);
	// out.block<3, 2>(12, 21) << -grav_matrix;
	out.block<3, 3>(3, 29) << Eigen::Matrix3d::Identity();				// wrt omega
	for (int ii = 0; ii < 3; ii++)								// wrt omega
	{
		Eigen::Vector3d de_delta_omega_de_axis = Eigen::Vector3d::Zero();
		de_delta_omega_de_axis(ii) = 1.0;
		out.block<3, 1>(30, 29 + ii) << -UAVmodel::inv_inertia * 
				( skew_symm(de_delta_omega_de_axis) * UAVmodel::inv_inertia * state.omega + skew_symm(state.omega) * UAVmodel::inv_inertia * de_delta_omega_de_axis );// +
				// UAVmodel::inv_inertia * (input.torque + state.btau);		// I need to consider also the (linear) exogenous input here!
	}
	// out.block<3, 3>(30, 32) << UAVmodel::inv_inertia;			// wrt btau
	return out;
}


Eigen::Matrix<double, 36, 12> df_dw(state_ikfom &s, const input_ikfom &in)
{
	// Eigen::Matrix<double, 30, 12> cov = Eigen::Matrix<double, 30, 12>::Zero();
	// cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
	// cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
	// cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
	// cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
	// cov.template bottomRightCorner(3, 3) = Eigen::Matrix3d::Identity();
	// return cov;
	Eigen::MatrixXd out;
	out.resize(36, 12);
	out.setZero();
	out.block<3, 3>(18, 0) << Eigen::Matrix3d::Identity();
	out.block<3, 3>(15, 3) << Eigen::Matrix3d::Identity();
	out.block<3, 3>(12, 6) << Eigen::Matrix3d::Identity();
	out.block<3, 3>(33, 9) << Eigen::Matrix3d::Identity();
	return out;
}
