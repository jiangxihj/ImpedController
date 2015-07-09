#ifndef __IMPED_CONTROL_H__
#define __IMPED_CONTROL_H__

//PR2 related
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <pr2_mechanism_model/robot.h>

//Boost smart pointer
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

//KDL related
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chaindynparam.hpp>

//realtime control related
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_box.h>

//ros related
#include <ros/ros.h>

//eigen related
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

//odeint related
#include "boost/numeric/odeint.hpp"

namespace imped_control_ns
{
class ImpedControlClass: public pr2_controller_interface::Controller
{

	// Declare the number of joints.
	enum
	{
		Joints = 7
	};

	// Define the joint/cart vector types accordingly (using a fixed
	// size to avoid dynamic allocations and make the code realtime safe).
	typedef Eigen::Matrix<double, Joints, 1>  JointVector;
	// typedef Eigen::Transform3d                CartPose;
	// typedef Eigen::Matrix<double, 3, 1>       Cart3Vector;
	typedef Eigen::Matrix<double, 6, 1>       Cart6Vector;
	typedef Eigen::Matrix<double, 6, Joints>  JacobianMatrix;
	typedef Eigen::Matrix<double, Joints, Joints>  JointInertiaMatrix;
	typedef Eigen::Matrix<double, 6, 6>  CartesianInertiaMatrix;

	// Ensure 128-bit alignment for Eigen
	// See also http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


private:
	/// The current robot state
	//(to get the time stamp)
	// Read-only after initialization
	pr2_mechanism_model::RobotState* robot_state_;

	/// The chain of links and joints in PR2 language for reference and commanding
	pr2_mechanism_model::Chain chain_;
	/// The chain of links and joints in PR2 language for reference only
	//Read-only after initialization
	pr2_mechanism_model::Chain read_only_chain_;
	/// The chain of links and joints in KDL language
	// Read-only after initialization
	KDL::Chain kdl_chain_;

	/// KDL Solver performing the joint angles to Cartesian pose calculation
	// Referenced only in update loop
	boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
	/// KDL Solver performing the joint angles to Jacobian calculation
	// Referenced only in update loop
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
	//KDL Solver to calculate the matrices M (inertia),C(coriolis) and G(gravitation) in joint space
	// Referenced only in update loop
	boost::scoped_ptr<KDL::ChainDynParam> MCG_solver_;

	/// Time of the last servo cycle
	//Modified in both command callback
	//and update loop, but it does not matter
	ros::Time last_time_;
	double dt;

	// The variables (which are declared as class variables
	// because they need to be pre-allocated, meaning we must
	// be very careful how we access them to avoid race conditions)
	/// Joint positions
	// Referenced only in update loop
	KDL::JntArray  q_;
	JointVector q_e;//variable represent using eigen definition
	/// Joint velocities
	// Referenced only in update loop
	KDL::JntArrayVel  qdot_;
	JointVector qdot_e;
	/// Joint torques
	// Referenced only in update loop
	// Command torque of task and posture movement
	KDL::JntArray tau_sum_;
	JointVector tau_task_e, tau_pose_e, tau_sum;//variable represent using eigen definition

	/// Tip pose
	// Referenced only in update loop
	KDL::Frame     x_;
	/// Tip desired pose
	// Referenced only in update loop
	KDL::Frame     xd_;

	/// Cartesian error
	// Referenced only in update loop
	KDL::Twist     xerr_;
	/// Cartesian velocity
	// Referenced only in update loop
	KDL::Twist     xdot_;
	Cart6Vector xdot_e;
	/// Cartesian effort
	// Referenced only in update loop
	KDL::Wrench    F_;
	/// Desired Cartesian force
	// Referenced only in update loop
	KDL::Wrench    Fdes_;
	/// Jacobian
	//Jacobian matrix in last moment to compute the derivate of jacobian ** to do: using finite difference coefficient(accuracy 3)
	KDL::Jacobian J_1_;
	// Referenced only in update loop
	KDL::Jacobian  J_;
	//Jacobian matrix in eigen form and dynamic consistent generlized inverse of jacobian
	JacobianMatrix J_e, J_bar_transpose, J_1_e, J_delta, J_dot;


	//Gravoty vector on end-effector
	KDL::Vector grav_;

	//Joint space inertia matrix
	KDL::JntSpaceInertiaMatrix M_;
	//Joint space inertia matrix in eigen form and its inverse
	JointInertiaMatrix M_e, M_inv;
	//Joint space coriolis matrix
	KDL::JntArray C_;
	//Joint space garvity torque
	KDL::JntArray G_;
	JointVector G_e;

	//Cartesian inertia matrix used in cartesian impedance control
	CartesianInertiaMatrix M_cart_inv, M_cart;

	//Dynamically consistent null space projection matrix
	//no define new matrix type, just use JointInertiaMatrix of same size
	JointInertiaMatrix null_proj;

	//Generlized moment
	JointVector p_e;//variable represent using eigen definition

	//Residual error
	std::vector<double> r_;
	JointVector r_e;

	//Desired inertia and damping matrix
	//no define new matrix type, just use JointInertiaMatrix of same size
	CartesianInertiaMatrix Mass_d, Damp_d;

	//KDL matrix & vector to eigen form
	void InertiaMatrixKDLToEigen(KDL::JntSpaceInertiaMatrix &m, JointInertiaMatrix &mass);
	void JacobianMatrixKDLToEigen(KDL::Jacobian &jac, JacobianMatrix &j_e);
	void ArrayKDLToEigen(KDL::JntArrayVel &a, JointVector &v);
	void ArrayKDLToEigen(KDL::JntArray &a, JointVector &v);
	void TwistKDLToEigen(KDL::Twist &t, Cart6Vector &v);
	//std::vector to eigen
	void VectorSTDToEigen(std::vector<double> &d, JointVector &v);

	//simple function to do pesudo inverse
	void PseudoInverse(CartesianInertiaMatrix const & a,
	                   double epsilon,
	                   CartesianInertiaMatrix & inv);

	//Eigen matrix to kdl vector, used in chain_.setEfforts
	void ArrayEigenToKDL(JointVector &v, KDL::JntArray &a);

	//----------------------------------define ode class to compute residual error-----------------------------------//
//ode class to compute residual error of force feedback using odeint
	class ResidualErrorClass {
		//command torque / gravity torque on each joint
		double tau_c, g_;
	public:
		ResidualErrorClass(double tau, double g): tau_c(tau), g_(g) {}
		ResidualErrorClass() {}
		void operator()( const double &x , double &dxdt , double t )
		{
			dxdt = tau_c - g_ + x;
		}
		//setter function for these two parameter
		void set_tau(int tau) {tau_c = tau;}
		void set_gravity(int g) {g_ = g;}

	};
	// state_type = double
	boost::numeric::odeint::runge_kutta_dopri5< double > stepper_type;

	// define vector of ode to computer all these seven joints
	std::vector<ResidualErrorClass> joint_residual;

public:

	bool init(pr2_mechanism_model::RobotState *robot,
	          ros::NodeHandle &n);

	void starting();

	void update();

	void stopping();
};
}

#endif
//ee_cart_imped_control.hpp