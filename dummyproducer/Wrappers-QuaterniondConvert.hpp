/*
 * Conversion functions for asn1SccWrappers_Quaterniond.
 */

#ifndef WRAPPERS_QUATERNIOND_CONVERT
#define WRAPPERS_QUATERNIOND_CONVERT

#include "baseTypes.h"
#include <base/wrappers/Eigen.hpp>
#include <base/wrappers/Eigen.hpp>
#include <string.h>
#include <stdio.h>


//Conversion functions from asn1 to c++ type
void asn1SccWrappers_Quaterniond_fromAsn1(wrappers::Quaternion<double>& result, const asn1SccWrappers_Quaterniond& asnVal);

//Conversion functions from C++ to Asn1 type
void asn1SccWrappers_Quaterniond_toAsn1(asn1SccWrappers_Quaterniond& result, const wrappers::Quaternion<double>& baseObj);


#endif //WRAPPERS_QUATERNIOND_CONVERT
