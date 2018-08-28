/*
 * Intermediate conversion functions.
 */

#ifndef OPAQUE_CONVERSION_H
#define OPAQUE_CONVERSION_H

#include "baseTypes.h"
#include <base/Eigen.hpp>
#include <base/wrappers/Eigen.hpp>

//Conversion functions from the instanced type to the marshaled type


//void Base_Affine3d_fromIntermediate(base::Affine3d& result, const wrappers::Matrix4d& cppMarshall);
//
//void Base_Affine3d_toIntermediate(wrappers::Matrix4d& result, const base::Affine3d& cppVal);
//
//void Base_Matrix4d_fromIntermediate(base::Matrix4d& result, const wrappers::Matrix4d& cppMarshall);
//
//void Base_Matrix4d_toIntermediate(wrappers::Matrix4d& result, const base::Matrix4d& cppVal);
//
//void Base_Vector4d_fromIntermediate(base::Vector4d& result, const wrappers::Vector4d& cppMarshall);
//
//void Base_Vector4d_toIntermediate(wrappers::Vector4d& result, const base::Vector4d& cppVal);
//
//void Base_AngleAxisd_fromIntermediate(base::AngleAxisd& result, const wrappers::AngleAxisd& cppMarshall);
//
//void Base_AngleAxisd_toIntermediate(wrappers::AngleAxisd& result, const base::AngleAxisd& cppVal);
//
//void Base_Matrix2d_fromIntermediate(base::Matrix2d& result, const wrappers::Matrix2d& cppMarshall);
//
//void Base_Matrix2d_toIntermediate(wrappers::Matrix2d& result, const base::Matrix2d& cppVal);
//
//void Base_VectorXd_fromIntermediate(base::VectorXd& result, const wrappers::VectorXd& cppMarshall);
//
//void Base_VectorXd_toIntermediate(wrappers::VectorXd& result, const base::VectorXd& cppVal);
//
//void Base_Vector6d_fromIntermediate(base::Vector6d& result, const wrappers::Vector6d& cppMarshall);
//
//void Base_Vector6d_toIntermediate(wrappers::Vector6d& result, const base::Vector6d& cppVal);
//
//void Base_Vector2d_fromIntermediate(base::Vector2d& result, const wrappers::Vector2d& cppMarshall);
//
//void Base_Vector2d_toIntermediate(wrappers::Vector2d& result, const base::Vector2d& cppVal);
//
//void Base_MatrixXd_fromIntermediate(base::MatrixXd& result, const wrappers::MatrixXd& cppMarshall);
//
//void Base_MatrixXd_toIntermediate(wrappers::MatrixXd& result, const base::MatrixXd& cppVal);

void Base_Matrix3d_fromIntermediate(base::Matrix3d& result, const wrappers::Matrix3d& cppMarshall);

void Base_Matrix3d_toIntermediate(wrappers::Matrix3d& result, const base::Matrix3d& cppVal);

//void Base_Matrix6d_fromIntermediate(base::Matrix6d& result, const wrappers::Matrix6d& cppMarshall);
//
//void Base_Matrix6d_toIntermediate(wrappers::Matrix6d& result, const base::Matrix6d& cppVal);

void Base_Vector3d_fromIntermediate(base::Vector3d& result, const wrappers::Vector3d& cppMarshall);

void Base_Vector3d_toIntermediate(wrappers::Vector3d& result, const base::Vector3d& cppVal);

void Base_Quaterniond_fromIntermediate(base::Quaterniond& result, const wrappers::Quaterniond& cppMarshall);

void Base_Quaterniond_toIntermediate(wrappers::Quaterniond& result, const base::Quaterniond& cppVal);

// Direct conversions for wrappers (fully manually coded)

void asn1Scc_Vector2d_fromAsn1(base::Vector2d& result, const asn1SccWrappers_Vector2d& asnVal);
void asn1Scc_Vector2d_toAsn1(asn1SccWrappers_Vector2d& result, const base::Vector2d& baseObj);

void asn1Scc_Vector3d_fromAsn1(base::Vector3d& result, const asn1SccWrappers_Vector3d& asnVal);
void asn1Scc_Vector3d_toAsn1(asn1SccWrappers_Vector3d& result, const base::Vector3d& baseObj);

void asn1Scc_Vector4d_fromAsn1(base::Vector4d& result, const asn1SccWrappers_Vector4d& asnVal);
void asn1Scc_Vector4d_toAsn1(asn1SccWrappers_Vector4d& result, const base::Vector4d& baseObj);

void asn1Scc_Vector6d_fromAsn1(base::Vector6d& result, const asn1SccWrappers_Vector6d& asnVal);
void asn1Scc_Vector6d_toAsn1(asn1SccWrappers_Vector6d& result, const base::Vector6d& baseObj);

void asn1Scc_VectorXd_fromAsn1(base::VectorXd& result, const asn1SccWrappers_VectorXd& asnVal);
void asn1Scc_VectorXd_toAsn1(asn1SccWrappers_VectorXd& result, const base::VectorXd& baseObj);

void asn1Scc_Matrix2d_fromAsn1(base::Matrix2d& result, const asn1SccWrappers_Matrix2d& asnVal);
void asn1Scc_Matrix2d_toAsn1(asn1SccWrappers_Matrix2d& result, const base::Matrix2d& baseObj);

void asn1Scc_Matrix3d_fromAsn1(base::Matrix3d& result, const asn1SccWrappers_Matrix3d& asnVal);
void asn1Scc_Matrix3d_toAsn1(asn1SccWrappers_Matrix3d& result, const base::Matrix3d& baseObj);

void asn1Scc_Matrix4d_fromAsn1(base::Matrix4d& result, const asn1SccWrappers_Matrix4d& asnVal);
void asn1Scc_Matrix4d_toAsn1(asn1SccWrappers_Matrix4d& result, const base::Matrix4d& baseObj);

void asn1Scc_Matrix6d_fromAsn1(base::Matrix6d& result, const asn1SccWrappers_Matrix6d& asnVal);
void asn1Scc_Matrix6d_toAsn1(asn1SccWrappers_Matrix6d& result, const base::Matrix6d& baseObj);

void asn1Scc_MatrixXd_fromAsn1(base::MatrixXd& result, const asn1SccWrappers_MatrixXd& asnVal);
void asn1Scc_MatrixXd_toAsn1(asn1SccWrappers_MatrixXd& result, const base::MatrixXd& baseObj);

void asn1Scc_Quaterniond_fromAsn1(base::Quaterniond& result, const asn1SccWrappers_Quaterniond& asnVal);
void asn1Scc_Quaterniond_toAsn1(asn1SccWrappers_Quaterniond& result, const base::Quaterniond& baseObj);

void asn1Scc_AngleAxisd_fromAsn1(base::AngleAxisd& result, const asn1SccWrappers_AngleAxisd& asnVal);
void asn1Scc_AngleAxisd_toAsn1(asn1SccWrappers_AngleAxisd& result, const base::AngleAxisd& baseObj);


#endif //OPAQUE_CONVERSION_H
