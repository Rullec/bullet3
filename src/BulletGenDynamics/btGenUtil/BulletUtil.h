#pragma once
#include "LinearMath/btMatrix3x3.h"
#include "MathUtil.h"
#include <iostream>
#include "BulletInverseDynamics/IDConfig.hpp"
#include "../Extras/InverseDynamics/btMultiBodyTreeCreator.hpp"

class btBulletUtil
{
public:
	static tVector btVectorTotVector0(const btVector3& v)
	{
		return tVector(v[0], v[1], v[2], 0);
	}
	static tVector btVectorTotVector1(const btVector3& v)
	{
		return tVector(v[0], v[1], v[2], 1);
	}
	static btVector3 tVectorTobtVector(const tVector& t)
	{
		return btVector3(t[0], t[1], t[2]);
	}
	static tMatrix btMatrixTotMatrix0(const btMatrix3x3& mat)
	{
		tMatrix res = tMatrix::Zero();
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++) res(i, j) = mat[i][j];
		return res;
	}
	static tMatrix btMatrixTotMatrix1(const btMatrix3x3& mat)
	{
		tMatrix res = tMatrix::Identity();
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++) res(i, j) = mat[i][j];
		return res;
	}
	static tQuaternion btQuaternionTotQuaternion(const btQuaternion& qua)
	{
		return tQuaternion(qua.w(), qua.x(), qua.y(), qua.z());
	}
	static btQuaternion tQuaternionTobtQuaternion(const tQuaternion& qua)
	{
		return btQuaternion(qua.x(), qua.y(), qua.z(), qua.w());
	}

	static Eigen::VectorXd btArrayToEigenArray(const btScalar* begin, const int size)
	{
		Eigen::VectorXd array(size);
		for (int i = 0; i < size; i++) array[i] = begin[i];
		return array;
	}
	static Eigen::VectorXd btArrayToEigenArray(const btAlignedObjectArray<btScalar>& info)
	{
		Eigen::VectorXd array(info.size());
		for (int i = 0; i < info.size(); i++) array[i] = info[i];
		return array;
	}
	static tVector btIDVectorTotVector0(const btInverseDynamicsBullet3::vec3& vec)
	{
		return tVector(vec(0), vec(1), vec(2), 0);
	}
	static Eigen::VectorXd btIDArrayToEigenArray(const btInverseDynamicsBullet3::vecx& vec)
	{
		Eigen::VectorXd res(vec.size());
		for (int i = 0; i < res.size(); i++) res[i] = vec(i);
		return res;
	}
	static btInverseDynamicsBullet3::vecx EigenArrayTobtIDArray(const tVectorXd& vec)
	{
		btInverseDynamicsBullet3::vecx res(vec.size());
		for (int i = 0; i < vec.size(); i++) res(i) = vec[i];
		return res;
	}
	static tVector btIDVectorTotVector1(const btInverseDynamicsBullet3::vec3& vec)
	{
		return tVector(vec(0), vec(1), vec(2), 1);
	}
	static tMatrix btIDMatrixTotMatrix(const btInverseDynamicsBullet3::mat33& mat)
	{
		tMatrix res = tMatrix::Identity();
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				res(i, j) = mat(i, j);

		return res;
	}

	static btMatrixXu tMatrixXdTobtMatrixX(const tMatrixXd& mat)
	{
		// TODO: alternative memcpy
		btMatrixXu res(mat.rows(), mat.cols());
		for (int i = 0; i < mat.rows(); i++)
			for (int j = 0; j < mat.cols(); j++) res.setElem(i, j, mat(i, j));
		return res;
	}

	static tMatrixXd btMatrixXTotMatrixXd(const btMatrixXu& mat)
	{
		// TODO: alternative memcpy
		tMatrixXd res(mat.rows(), mat.cols());
		for (int i = 0; i < mat.rows(); i++)
			for (int j = 0; j < mat.cols(); j++) res(i, j) = mat(i, j);
		return res;
	}

	static btVectorXu tVectorXdTobtVectorXd(const tVectorXd& vec)
	{
		btVectorXu res(vec.size());
		for (int i = 0; i < vec.size(); i++) res[i] = vec[i];
		return res;
	}

	static tVectorXd btVectorXdTotVectorXd(const btVectorXu& vec)
	{
		tVectorXd res(vec.size());
		for (int i = 0; i < vec.size(); i++) res[i] = vec[i];
		return res;
	}

	static btMatrix3x3 AsDiagnoal(const btVector3& vec)
	{
		btMatrix3x3 mat = btMatrix3x3::getIdentity();
		for (int i = 0; i < 3; i++) mat[i][i] = vec[i];
		return mat;
	}

	template <typename T>
	static void PrintBulletVars(const T& mat)
	{
		int rows = mat.rows(),
			cols = mat.cols();
		if (typeid(T) == typeid(btVectorXu))
		{
			cols = rows;
			rows = 1;
		}
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				std::cout << mat(i, j) << " ";
			}
			std::cout << std::endl;
		}
	}
};