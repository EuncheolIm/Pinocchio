#include "robot.h"
 
// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "/Users/im-euncheol/Desktop/KIST/CBFs/model"
#endif
 
PModel::PModel()
{
  Initialize();
}
PModel::~PModel()
{
}
void PModel::Initialize()
{
  _k = 7;
  _q.setZero(_k);
  _qdot.setZero(_k);
  load_model();
  offset = true;

  _J_hand.setZero(6,_k);
  _xdot_hand.setZero(6);
  _M.setZero(_k,_k);

  v.setZero(_k);
  a.setZero(_k);
  _b.setZero(_k);
  _g.setZero(_k);
}
void PModel::load_model()
{   
	const std::string urdf_filename =  PINOCCHIO_MODEL_DIR + std::string("/fr3.urdf");
	pinocchio::urdf::buildModel(urdf_filename,_model);
  cout << endl << endl << "Model Loaded for Pinocchio." << endl << "Total DoFs: " << _model.nv << endl << endl;
  std::cout << "model name: " << _model.name << std::endl<<std::endl;

  _data = pinocchio::Data(_model); // Initialize _data
  
  if (_model.nv != _k)
	{
		cout << "Simulation model and RBDL model mismatch!!!" << endl << endl;
	}

  _bool_model_update = true; //check model update
	cout << "Model Loading Complete." << endl << endl;

}
void PModel::update_kinematics(VectorXd & q, VectorXd & qdot)
{
  _q = q;
	_qdot = qdot;

  if (offset == true)
  {
    pinocchio::SE3 eeFramePlacement(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0.211));
    int parentJointId = _model.getJointId("panda_joint7");

    if(parentJointId < 0 || parentJointId >= _model.njoints) {
      std::cerr << "Invalid parent joint index." << std::endl;
    }

    _model.addFrame(pinocchio::Frame("end_effector", parentJointId, 0, eeFramePlacement, pinocchio::OP_FRAME));
    end_effector_name = "end_effector";
  }
  else
  {
    end_effector_name = "panda_link7";
  }
  
  pinocchio::forwardKinematics(_model, _data, _q, _qdot);
  pinocchio::FrameIndex ee_frame_id = _model.getFrameId(end_effector_name);
  pinocchio::updateFramePlacements(_model, _data);
  
  // Get end-effector position and orientation
  _x_hand = _data.oMf[ee_frame_id].translation();
  _R_hand = _data.oMf[ee_frame_id].rotation();
  _x_hand_rpy = _R_hand.eulerAngles(2, 1, 0); // Order: yaw (Z), pitch (Y), roll (X)

  // std::cout <<"pos: " << _x_hand.transpose() << std::endl;
  // std::cout <<"ori: " << _x_hand_rpy.transpose() << std::endl;
  _bool_kinematics_update = true; // check kinematics update
}
void PModel::calculate_EE_Jacobians()
{

  if (_bool_kinematics_update == true)
	{
    pinocchio::Data::Matrix6x J(6, _k);
    _J_hand.setZero();
    pinocchio::computeJointJacobians(_model, _data, _q);

    try
    {
        pinocchio::Model::FrameIndex link_number = _model.getFrameId(end_effector_name);
        pinocchio::getFrameJacobian(_model, _data, link_number, pinocchio::LOCAL, J);
        // Transform Jacobians from pinocchio::LOCAL frame to base frame
        J.topRows(3) = (_data.oMf[link_number].rotation()) * J.topRows(3);
        J.bottomRows(3) = (_data.oMf[link_number].rotation()) * J.bottomRows(3);
        _J_hand = J;
        _bool_Jacobian_update = true;
        
    }
    catch (std::exception &e)
    {
        std::cerr << "WARNING: Link name " << end_effector_name << " is invalid! ... "
                  << "Returning zeros" << std::endl;
    }
  }
  else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}

}
void PModel::update_dynamics()
{
  if (_bool_kinematics_update == true)
	{
    pinocchio::crba(_model, _data, _q); //Composite Rigid Body Algorithm (CRBA)
    _M = _data.M;
    Eigen::MatrixXd M_full = _M.selfadjointView<Eigen::Upper>();
    // std::cout << "Full Inertia Matrix: \n" << M_full << std::endl<<endl;

    v = Eigen::VectorXd::Zero(_model.nv); // 관절 가속도
    a = Eigen::VectorXd::Zero(_model.nv); // 관절 가속도
    _b = pinocchio::rnea(_model, _data, _q, _qdot, a);

    // pinocchio::computeGeneralizedGravity(_model, _data, _q);
    // _g = pinocchio::rnea(_model, _data, _q, v, a);
    _g = _data.g;

    // pinocchio::nonLinearEffects(_model, _data, _q, _qdot);

    // _g = _data.nle;

    cout << _g.transpose() <<endl;
    // cout << _b.transpose() <<endl;

    // cout <<"m:"<< _M<<endl<<endl;
  }
  else
  {
    cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
  }
}

void PModel::calculate_EE_velocity()
{
  if (_bool_Jacobian_update == true)
	{
		_xdot_hand = _J_hand * _qdot;
	}
	else
	{
		cout << "Jacobian matrices are not ready. Please calculate Jacobians first." << endl << endl;
	}
}