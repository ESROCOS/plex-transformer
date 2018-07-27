/* User code: This file will not be overwritten by TASTE. */

#include "tools/transformer/Transformer.h"
#include "transformer.h"
#include "base_support/Base-samples-RigidBodyStateConvert.hpp"
#include "base_support/OpaqueConversion.hpp"
#include <Eigen/Core>
#include <iostream>

typedef esrocos::transformer::AcyclicTransformer<3> atf;

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
void to4d(Eigen::Matrix3d r, Eigen::Vector3d t, Eigen::Matrix4d & h){
	
  h<< r(0,0), r(0,1), r(0,2), t[0],
      r(1,0), r(1,1), r(1,2), t[1],
      r(2,0), r(2,1), r(2,2), t[2],
          0,      0,      0,    1;
}

void split4d(Eigen::Matrix4d h, Eigen::Matrix3d & r, Eigen::Vector3d & t){
 
  
  r << h(0,0), h(0,1), h(0,2),
       h(1,0), h(1,1), h(1,2), 
       h(2,0), h(2,1), h(2,2); 
  
  t << h(0,3),h(1,3),h(2,3);
}

void transformer_startup()
{
  atf::Transformation odom, camera_fixture;
  atf::Frame world, robot_base, camera_1;
  Eigen::Matrix4d identity, fixture_init;

  // Matrices
  identity << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
	      0, 0, 0, 1;

  fixture_init << 1, 0, 0, 0,
	          0, 1, 0, 0,
		  0, 0, 1, 1,
		  0, 0, 0, 1;

  // TRANSFORMATIONS
  odom = atf::Transformation(world,robot_base,"odom");
    odom.atob(identity);

  camera_fixture = atf::Transformation(robot_base,camera_1,"camera_fixture");
    camera_fixture.atob(fixture_init);


  // FRAMES
  world = atf::Frame("world");
  
  robot_base = atf::Frame("robot_base");
    robot_base.transformToParent = odom;
 
  camera_1 = atf::Frame("camera_1"); 
    camera_1.transformToParent = camera_fixture;

  // TRANSFORM TREE

  TF = atf(world);

    TF.addFrame(robot_base);
    TF.addFrame(camera_1);
    
    std::cout << "setup success" << std::endl;
/*
    atf::Transformation t0, t1;
    atf::Frame f0,f1;

    std::cout << "got base " << TF.getFrame("robot_base", f0) << std::endl;   
    std::cout << "got camera " << TF.getFrame("camera_1", f1) << std::endl;   

    t0 = f0.transformToParent;
    t1 = f1.transformToParent;

    std::cout << "base to world\n" << t0.atob() << std::endl;    
    std::cout << "camera to base\n" << t1.atob() << std::endl;    
*/
}

void transformer_PI_robotPose(const asn1SccBase_samples_RigidBodyState *IN_pose)
{
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
}

void transformer_PI_relativeMarkerPose(const asn1SccBase_samples_RigidBodyState *IN_pose)
{
  base::Vector3d t;
  base::Quaterniond q;
  Eigen::Matrix4d inPose;

  // extract orientation / translation
  asn1Scc_Vector3d_fromAsn1(t, IN_pose->position);
  asn1Scc_Quaterniond_fromAsn1(q, IN_pose->orientation);

  Eigen::Matrix3d r = q.normalized().toRotationMatrix();
  
  to4d(r,t,inPose);
  
  // get current transform t1 from camera to world
  Eigen::Matrix4d toWorld;
  bool gotTransform = TF.getTransform("camera_1","world",toWorld);

  //std::cout << "toWorld:\n" << toWorld << std::endl;

  // multiply t1 * t0 to get pose in world frame

  Eigen::Matrix4d globalPose = toWorld * inPose;

  //std::cout << "t2:\n" << t2 << std::endl;
  
  Eigen::Vector3d _t;
  Eigen::Matrix3d _r;

  split4d(globalPose,_r,_t);

  q = base::Quaterniond(r);
  
  /*
  std::cout << "\n" << _r << "\n" << std::endl;
  std::cout << "\n" << _t << "\n" << std::endl;
  */
 
  // write out pose in world frame
  asn1Scc_Vector3d_toAsn1(OUT_pose.position, _t);
  asn1Scc_Quaterniond_toAsn1(OUT_pose.orientation, q);
  std::cout << "call marker pose:" << std::endl;
  transformer_RI_absoluteMarkerPose(&OUT_pose);
}

