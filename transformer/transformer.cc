/* User code: This file will not be overwritten by TASTE. */

#include "tools/transformer/Transformer.h"
#include "transformer.h"
#include "base_support/Base-samples-RigidBodyStateConvert.hpp"
#include "base_support/OpaqueConversion.hpp"
#include <Eigen/Core>
//#include <iostream>

typedef esrocos::transformer::AcyclicTransformer<3> atf;

void set_nonstandard_mode()
{
    uint32_t fsrVal;
    uint32_t *fsrValPtr = &fsrVal;
    __asm__(
	"st %%fsr, [%0]"
	:
	: "r" (fsrValPtr)
	: "memory"
    );
    fsrVal |= (1 << 22); // Set NS Bit
    __asm__(
        "ld [%0], %%fsr"
        :
        : "r" (fsrValPtr)
	: "memory"
    );
}

/*
The following tree is modelled for testing.

[Frame]
(dynamic transformation)
<static transformation>

    [world]
       |
   (odometry)
       |
  [robot_base]
       |
<camera_fixture> 
       |
   [camerai_1]
*/

atf TF("world");
asn1SccBase_samples_RigidBodyState OUT_pose;

// EIGEN HELPER METHODS
//double dround(double x) // the functor we want to apply
//{ 
//  int i = (int)(x*10000.0);
//  double r = (double)i / 10000.0;
//  return r;
//}
//
//Eigen::Matrix4d roundElements(Eigen::Matrix4d m){
//  Eigen::Matrix4d n = m;
//  n = m.unaryExpr(&dround);
//  return n;
//}

void to4d(Eigen::Matrix3d r, Eigen::Vector3d t, Eigen::Matrix4d & h){
	
  h<< r(0,0), r(0,1), r(0,2), t[0],
      r(1,0), r(1,1), r(1,2), t[1],
      r(2,0), r(2,1), r(2,2), t[2],
          0.,      0.,      0.,    1.;
}

void split4d(Eigen::Matrix4d h, Eigen::Matrix3d & r, Eigen::Vector3d & t){
 
  
  r << h(0,0), h(0,1), h(0,2),
       h(1,0), h(1,1), h(1,2), 
       h(2,0), h(2,1), h(2,2); 
  
  t << h(0,3),h(1,3),h(2,3);
}

void transformer_startup()
{
  set_nonstandard_mode();
  atf::Transformation odom, camera_fixture;
  atf::Frame world("world"),robot_base("robot_base"), camera_1("camera_1");
  Eigen::Matrix4d identity, fixture_init;

  // Matrices
  identity << 1., 0., 0., 0.,
              0., 1., 0., 0.,
              0., 0., 1., 0.,
	      0., 0., 0., 1.;

  fixture_init << 1., 0., 0., 0.,
	          0., 1., 0., 0.,
		  0., 0., 1., 1.,
		  0., 0., 0., 1.;

  // TRANSFORMATIONS
  odom = atf::Transformation(robot_base,world,"odom");
    odom.atob(identity);

  camera_fixture = atf::Transformation(camera_1,robot_base,"camera_fixture");
    camera_fixture.atob(fixture_init);

  // FRAMES
  robot_base.transformToParent = odom;
 
  camera_1.transformToParent = camera_fixture;

  // TRANSFORM TREE

  TF = atf(world);

    TF.addFrame(robot_base);
    TF.addFrame(camera_1);
    
  //TF.printAdresses();

  //std::cout << "transformer: tf tree setup success" << std::endl;
}

void transformer_PI_robotPose(const asn1SccBase_samples_RigidBodyState *IN_pose)
{
  set_nonstandard_mode();
//  std::cout << "transformer: got robot pose" << std::endl;
  // extract orientation / translation
  // write to matrix4f
  // update robot pose with matrix4f
  base::Vector3d t;
  base::Quaterniond q;
  Eigen::Matrix4d pose;

  // extract orientation / translation
  asn1Scc_Vector3d_fromAsn1(t, IN_pose->position);
  asn1Scc_Quaterniond_fromAsn1(q, IN_pose->orientation);
  // convert quaternion to rotation mat
  Eigen::Matrix3d r = q.normalized().toRotationMatrix();

  to4d(r,t,pose);
  TF.updateTransform("odom",pose);

  //std::cout << "transformer: updated robot pose" << std::endl;
  //std::cout << roundElements(pose) << std::endl;
}

void transformer_PI_relativeMarkerPose(const asn1SccBase_samples_RigidBodyState *IN_pose)
{
  set_nonstandard_mode();

  base::Vector3d t;
  base::Quaterniond q;
  Eigen::Matrix4d inPose;

  // extract orientation / translation
  asn1Scc_Vector3d_fromAsn1(t, IN_pose->position);
  asn1Scc_Quaterniond_fromAsn1(q, IN_pose->orientation);

  Eigen::Matrix3d r = q.normalized().toRotationMatrix();
  
  to4d(r,t,inPose);

  //std::cout << "got marker pose" << std::endl;
  //std::cout << roundElements(inPose) << std::endl;
  
  atf::Transformation odom, camera_fixture;

  TF.getTransform("odom", odom);
  TF.getTransform("camera_fixture", camera_fixture);

  //std::cout << "camera_fixture\n" << roundElements(camera_fixture.atob()) <<  std::endl;
  //std::cout << "odom\n" << roundElements(odom.atob()) << std::endl;

  // get current transform t1 from camera to world
  Eigen::Matrix4d toWorld;
  bool gotTransform = TF.getTransform("camera_1","world",toWorld);
  
  //std::cout << "manual toWorld (odom * camera_fixture):\n" << roundElements(odom.atob() * camera_fixture.atob()) << std::endl;
  //std::cout << "calculated toWorld:\n" << roundElements(toWorld) << std::endl;

  if (!gotTransform)
	return;

  // multiply t1 * t0 to get pose in world frame

  Eigen::Matrix4d globalPose = toWorld * inPose;
  
  Eigen::Vector3d _t;
  Eigen::Matrix3d _r;

  split4d(globalPose,_r,_t);

  q = base::Quaterniond(r);
 
  // write out pose in world frame
  asn1Scc_Vector3d_toAsn1(OUT_pose.position, _t);
  asn1Scc_Quaterniond_toAsn1(OUT_pose.orientation, q);
  transformer_RI_absoluteMarkerPose(&OUT_pose);

  //std::cout << "calculated marker pose (toWorld * inPose)" << std::endl;
  //std::cout << roundElements(globalPose) << std::endl;
}

