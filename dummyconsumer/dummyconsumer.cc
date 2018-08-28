/* User code: This file will not be overwritten by TASTE. */

#include "dummyconsumer.h"
#include <iostream>
#include "base_support/Base-samples-RigidBodyStateConvert.hpp"
#include "base_support/OpaqueConversion.hpp"

void dummyconsumer_startup()
{
    /* Write your initialization code here,
       but do not make any call to a required interface. */
}

void dummyconsumer_PI_consumeDummy(const asn1SccBase_samples_RigidBodyState *IN_pose)
{
  std::cout << "x: " << IN_pose->position.data.arr[0] << " y: " << IN_pose->position.data.arr[1] << " z: " << IN_pose->position.data.arr[2] << "\n";
}

