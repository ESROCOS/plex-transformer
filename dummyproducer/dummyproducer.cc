/* User code: This file will not be overwritten by TASTE. */

#include "dummyproducer.h"
#include "base_support/Base-samples-RigidBodyStateConvert.hpp"
#include "base_support/OpaqueConversion.hpp"
#include <Eigen/Core>
#include <iostream>
#include <cmath>

#define _USE_MATH_DEFINES

base::Vector3d t0(3,0,0);
base::Vector3d t1(2,2,0);
base::Vector3d t2(0,3,0);
base::Vector3d t3(-2,2,0);
base::Vector3d t4(-3,0,0);
base::Vector3d t5(-2,-2,0);
base::Vector3d t6(0,-3,0);
base::Vector3d t7(2,-2,0);

base::Vector3d positions[8] = {t0,t1,t2,t3,t4,t5,t6,t7};

Eigen::Matrix3d r;

asn1SccBase_samples_RigidBodyState robotPose;
asn1SccBase_samples_RigidBodyState markerPose;

void setRot(double x){
  r << cos(x), -sin(x), 0,
       sin(x),  cos(x), 0,
            0,       0, 1;	  
}

void dummyproducer_startup()	
{
  r << 1,0,0,
       0,1,0,
       0,0,1;
}

void dummyproducer_PI_clock()
{
  static int i = 0;
  std::cout << "producer: clock tick " << i << std::endl;

  if (i % 2 == 0) {
    double x = ((i/2) % 8) * 0.25 * M_PI;
    setRot(x);

    //std::cout << "rot matrix:\n" << r << std::endl;
    base::Quaterniond q(r);

    asn1Scc_Vector3d_toAsn1(robotPose.position, positions[(i/2)%8]);
    asn1Scc_Quaterniond_toAsn1(robotPose.orientation, q);
  
    std::cout << "producer: push robot pose" << std::endl;
    dummyproducer_RI_robotPose(&robotPose);
  }

  if ((i-1) % 4 == 0) {

    Eigen::Matrix3d r1;
    r1 << 1,0,0,
          0,1,0,
          0,0,1;

    base::Quaterniond q1(r1);

    base::Vector3d t1;
    t1 << 2,0,0;
   
    
    asn1Scc_Vector3d_toAsn1(markerPose.position, t1);  
    asn1Scc_Quaterniond_toAsn1(markerPose.orientation, q1);
    std::cout << "producer: push marker pose" << std::endl;
    dummyproducer_RI_relativeMarkerPose(&markerPose);  
  }
  
  i++;
}

