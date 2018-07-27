/* User code: This file will not be overwritten by TASTE. */

#include "dummyproducer.h"
#include "base_support/Base-samples-RigidBodyStateConvert.hpp"
#include "base_support/OpaqueConversion.hpp"
#include <Eigen/Core>
#include <iostream>

void dummyproducer_startup()
{
    /* Write your initialization code here,
       but do not make any call to a required interface. */
}

void dummyproducer_PI_clock()
{
  static int i = 0;

  i++;
  
  Eigen::Matrix3d r;
  r << 1,0,0,
       0,1,0,
       0,0,1;

  base::Quaterniond q(r);

  base::Vector3d t;
  t << 0,0,i;

  asn1SccBase_samples_RigidBodyState robotPose;

  asn1Scc_Vector3d_toAsn1(robotPose.position, t);
  asn1Scc_Quaterniond_toAsn1(robotPose.orientation, q);

  std::cout << "robot pose:\n" << t << std::endl;

  dummyproducer_RI_robotPose(&robotPose);

  if (i % 4 == 0) {
    asn1SccBase_samples_RigidBodyState markerPose;

    Eigen::Matrix3d r1;
    r1 << 1,0,0,
          0,1,0,
          0,0,1;

    base::Quaterniond q1(r1);

    base::Vector3d t1;
    t1 << 0,1,0;
    
    asn1Scc_Vector3d_toAsn1(markerPose.position, t1);  
    asn1Scc_Quaterniond_toAsn1(markerPose.orientation, q1);

    dummyproducer_RI_relativeMarkerPose(&markerPose);  
  }
}

