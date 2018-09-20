/* User code: This file will not be overwritten by TASTE. */

#include "tools/transformer/Transformer.h"
#include "transformer.h"
#include "base_support/Base-samples-RigidBodyStateConvert.hpp"
#include "base_support/OpaqueConversion.hpp"
#include <Eigen/Core>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>

//#define DEBUG
#define FPU_NON_STANDARD

typedef esrocos::transformer::AcyclicTransformer<3> atf;

void set_nonstandard_mode()
{
#ifdef FPU_NON_STANDARD
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
#endif
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

void init_rbs(asn1SccBase_samples_RigidBodyState *rbs)
{
   memset(rbs, 0, sizeof(asn1SccBase_samples_RigidBodyState));
   rbs->position.data.nCount = 3;
   rbs->cov_position.data.nCount = 9;
   rbs->orientation.im.nCount = 3;
   rbs->cov_orientation.data.nCount = 9;
   rbs->velocity.data.nCount = 3;
   rbs->cov_velocity.data.nCount = 9;
   rbs->angular_velocity.data.nCount = 3;
   rbs->cov_angular_velocity.data.nCount = 9;
}

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
  std::cout << "[transformer_startup] startup\n";
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

  // Initialize rigid body state
  init_rbs(&OUT_pose);
}

void transformer_PI_robotPose(const asn1SccBase_samples_RigidBodyState *IN_pose)
{
  set_nonstandard_mode();
  // extract orientation / translation
  // write to matrix4f
  // update robot pose with matrix4f
  base::Vector3d t;
  base::Quaterniond q;
  Eigen::Matrix4d pose;

  // extract orientation / translation
  asn1Scc_Vector3d_fromAsn1(t, IN_pose->position);
  asn1Scc_Quaterniond_fromAsn1(q, IN_pose->orientation);
#ifdef DEBUG
  std::cout << "[transformer_PI_robotPose] pos: " << t.transpose() << " orient: " << q.vec().transpose() << std::endl;
#endif
  // convert quaternion to rotation mat
  Eigen::Matrix3d r = q.normalized().toRotationMatrix();

  to4d(r,t,pose);
  TF.updateTransform("odom",pose);
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
#ifdef DEBUG
  std::cout << "[transformer_PI_relativeMarkerPose] pos: " << t.transpose() << " orient: " << q.vec().transpose() << std::endl;
#endif

  Eigen::Matrix3d r = q.normalized().toRotationMatrix();
  
  to4d(r,t,inPose);

  atf::Transformation odom, camera_fixture;

  TF.getTransform("odom", odom);
  TF.getTransform("camera_fixture", camera_fixture);

  // get current transform t1 from camera to world
  Eigen::Matrix4d toWorld;
  bool gotTransform = TF.getTransform("camera_1","world",toWorld);
  
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
#ifdef DEBUG
  std::cout << "[transformer_RI_absoluteMarkerPose] pos: " << _t.transpose() << " orient: " << q.vec().transpose() << std::endl;
#endif

  // set timestamp
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  OUT_pose.time.microseconds = spec.tv_nsec / 1000 + spec.tv_sec * 1000000;

  // Copy the id of the marker from incoming source frame to outgoing source frame string
  OUT_pose.sourceframe.nCount = snprintf((char*)OUT_pose.sourceframe.arr, IN_pose->sourceframe.nCount, "%s", (const char *)IN_pose->sourceframe.arr);

  transformer_RI_absoluteMarkerPose(&OUT_pose);
}

