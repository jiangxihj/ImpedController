#include "imped_control/imped_control.h"
using namespace imped_control_ns;

void ImpedControlClass::InertiaMatrixKDLToEigen(KDL::JntSpaceInertiaMatrix &m, JointInertiaMatrix &mass)
{
	for (int i = 0; i < m.data.rows(); i++)
	{
		for (int j = 0; j < m.data.cols(); j++)
		{
			mass(i, j) = m.data(i, j);
		}
	}
}

void ImpedControlClass::JacobianMatrixKDLToEigen(KDL::Jacobian &jac, JacobianMatrix &j_e)
{
	for (int i = 0; i < jac.data.rows(); i++)
	{
		for (int j = 0; j < jac.data.cols(); j++)
		{
			j_e(i, j) = jac.data(i, j);
		}
	}
}

void ImpedControlClass::ArrayKDLToEigen(KDL::JntArrayVel &a, JointVector &v)
{
	for (int i = 0; i < a.qdot.data.rows(); i++)
	{
		for (int j = 0; j < a.qdot.data.cols(); j++)
		{
			v(i, j) = a.qdot.data(i, j);
		}
	}
}

void ImpedControlClass::ArrayKDLToEigen(KDL::JntArray &a, JointVector &v)
{
	for (int i = 0; i < a.data.rows(); i++)
	{
		for (int j = 0; j < a.data.cols(); j++)
		{
			v(i, j) = a.data(i, j);
		}
	}
}

/**
   This pseudo-inverse is based on SVD, followed by threshlding on
   the singular values. This is a bit simplistic, but we have found
   that it works allright for our use cases with a sigmaThreshold of
   1e-4 or 1e-3.
*/
void ImpedControlClass::PseudoInverse(CartesianInertiaMatrix const & a,
                                      double epsilon,
                                      CartesianInertiaMatrix & inv)
{
	Eigen::JacobiSVD<CartesianInertiaMatrix> svd(a , Eigen::ComputeFullU | Eigen::ComputeFullV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
	inv = svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

void ImpedControlClass::TwistKDLToEigen(KDL::Twist &t, Cart6Vector &v)
{
	for (int i = 0; i < 6; i++)
	{
		v[i] = t[i];
	}
}

void ImpedControlClass::VectorSTDToEigen(std::vector<double> &d, JointVector &v)
{
	for (int i = 0; i < d.size(); i++)
	{
		v[i] = d[i];
	}
}

void ImpedControlClass::ArrayEigenToKDL(JointVector &v, KDL::JntArray &a)
{
	for (int i = 0; i < Joints; i++)
	{
		a.data(i) = v[i];
	}
}