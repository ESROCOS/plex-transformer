/*
 * Conversion functions for asn1SccWrappers_Matrix3d.
 */

#ifndef WRAPPERS_MATRIX3D_CONVERT
#define WRAPPERS_MATRIX3D_CONVERT

#include "baseTypes.h"
#include <base/wrappers/Eigen.hpp>
#include <base/wrappers/Eigen.hpp>
#include <string.h>
#include <stdio.h>


//Conversion functions from asn1 to c++ type
void asn1SccWrappers_Matrix3d_fromAsn1(wrappers::Matrix<double,3,3>& result, const asn1SccWrappers_Matrix3d& asnVal);

//Conversion functions from C++ to Asn1 type
void asn1SccWrappers_Matrix3d_toAsn1(asn1SccWrappers_Matrix3d& result, const wrappers::Matrix<double,3,3>& baseObj);


#endif //WRAPPERS_MATRIX3D_CONVERT
