#pragma once
#ifndef __ROBOT_H
#define __ROBOT_H

#include <iostream>
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
// #include <pinocchio/algorithm/velocity.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>



// #include <pinocchio/algorithm/gravity.hpp>
 
#include "custommath.h"


using namespace std;
using namespace Eigen;
// using namespace pinocchio;

class PModel
{
public:
	PModel();
	virtual ~PModel();

    pinocchio::Model _model;
    pinocchio::Data _data; // Added data as a member variable

    void update_kinematics(VectorXd & q, VectorXd & qdot); // update robot state
    void update_dynamics(); // calculate _A, _g, _b, _bg
    void calculate_EE_Jacobians(); // calcule jacobian
    void calculate_EE_positions_orientations(); // calculte End-effector postion, orientation
    void calculate_EE_velocity(); // calculate End-effector velocity

    MatrixXd _A; // inertia matrix
    VectorXd _g; // gravity force vector
	VectorXd _b; // Coriolis/centrifugal force vector

    MatrixXd _J_hand; // jacobian Matrix 6x7

    MatrixXd _hand_ori;

    Vector3d _position_local_task_hand; // End-effector coordinate
    Vector3d _x_hand; // End-effector position
    Vector3d _x_hand_rpy; // End-effector position
    Matrix3d _R_hand; // End-effector rotation matrix

    VectorXd _xdot_hand;
    MatrixXd _M; // Inertia matrix


    void load_model(); // read URDF model
private:
	void Initialize();
	// // void load_model(); // read URDF model
	void set_robot_config();

    VectorXd _q, _qdot; // joint 
    std::string end_effector_name; 
    bool offset;
    // VectorXd _zero_vec_joint; // zero joint vector

    int _k; // joint number
    // int _id_hand; // hand id
    // int _id_ee;// end-effector id
    VectorXd v;
    VectorXd a;

    bool _bool_model_update, _bool_kinematics_update, _bool_dynamics_update, _bool_Jacobian_update; // update check

};

#endif