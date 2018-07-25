/* User code: This file will not be overwritten by TASTE. */

#include "dummyproducer.h"

void dummyproducer_startup()
{
    /* Write your initialization code here,
       but do not make any call to a required interface. */
}

void dummyproducer_PI_clock()
{
  asn1SccBase_samples_RigidBodyState bs;



 
  dummyproducer_RI_relativeMarkerPose(&bs);  
}

