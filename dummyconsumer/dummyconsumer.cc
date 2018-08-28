/* User code: This file will not be overwritten by TASTE. */

#include "dummyconsumer.h"
#include <iostream>
//#include "base_support/Base-samples-RigidBodyStateConvert.hpp"
//#include "base_support/OpaqueConversion.hpp"

void dummyconsumer_startup()
{
    /* Write your initialization code here,
       but do not make any call to a required interface. */
}

void dummyconsumer_PI_consumeDummy(const asn1SccBase_samples_RigidBodyState *IN_pose)
{
  //base::Vector3d t;
  //asn1Scc_Vector3d_fromAsn1(t, IN_pose->position);
}

