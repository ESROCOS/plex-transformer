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
void transformer_startup()
{
atf::Transformation odom, camera_fixture;
atf::Frame world, robot_base, camera_1;
Eigen::Matrix4f identity, fixture_init;

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

    TF.addFrame(world);
    TF.addFrame(robot_base);
    TF.addFrame(camera_1);
    
  Eigen::Matrix4f t0;
    std::cout << "cam: " << camera_1.transformToParent.atob() << std::endl;    

    std::cout << "odom:\n" << t0 << std::endl;  

    if (TF.getTransform("camera_1","world",t0)){
      std::cout << "got transform: success" << std::endl;
    } else{
      std::cout << "got transform: fail" << std::endl;
    }

    std::cout << "inital transform camera_1 to world\n"<<  t0 << std::endl;
}

void transformer_PI_robotPose(const asn1SccBase_samples_RigidBodyState *IN_pose)
{
  // extract orientation / translation
  // write to matrix4f
  // update robot pose with matrix4f
  base::Vector3d t;
  base::Quaterniond q;

  // extract orientation / translation
  asn1Scc_Vector3d_fromAsn1(t, IN_pose->position);
  asn1Scc_Quaterniond_fromAsn1(q, IN_pose->orientation);

  Eigen::Matrix3d _r = q.normalized().toRotationMatrix();
  Eigen::Matrix3f r = _r.cast<float>();
  Eigen::Matrix4f _t;
   
  //std::cout << "got rotation:\n" << r << "\n\n" << t << std::endl;

  _t << r(0,0), r(1,0), r(2,0), t[0],
        r(0,1), r(1,1), r(2,1), t[1],
	r(0,2), r(1,2), r(2,2), t[2],
	     0,      0,      0,    1;

  TF.updateTransform("odom",_t);
}

void transformer_PI_relativeMarkerPose(const asn1SccBase_samples_RigidBodyState *IN_pose)
{
  base::Vector3d t;
  base::Quaterniond q;

  // extract orientation / translation
  asn1Scc_Vector3d_fromAsn1(t, IN_pose->position);
  asn1Scc_Quaterniond_fromAsn1(q, IN_pose->orientation);

  Eigen::Matrix3d _r = q.normalized().toRotationMatrix();
  Eigen::Matrix3f r = _r.cast<float>();
  Eigen::Matrix4f _t;
   
  //std::cout << "got rotation:\n" << r << "\n\n" << t << std::endl;

  _t << r(0,0), r(1,0), r(2,0), t[0],
        r(0,1), r(1,1), r(2,1), t[1],
	r(0,2), r(1,2), r(2,2), t[2],
	     0,      0,      0,    1;
  
  // write to matrix4f _t
 
  std::cout << "_t:\n" << _t << std::endl;  
  
  // get current transform t1 from camera to world
  Eigen::Matrix4f t1;
  bool gotTransform = TF.getTransform("camera_1","world",t1);

  std::cout << "gotTransform: " << gotTransform << std::endl;
  std::cout << "t1:\n" << t1 << std::endl;
  // multiply t1 * t0 to get pose in world frame

  Eigen::Matrix4f t2 = t1 * _t;

  std::cout << "t2:\n" << t2 << std::endl;

  r << t2(0,0), t2(1,0), t2(2,0),
       t2(0,1), t2(1,1), t2(2,1), 
       t2(0,2), t2(1,2), t2(2,2); 
  
  _r = r.cast<double>();

  q = base::Quaterniond(_r);

  t << t2(3,0),t2(3,1),t2(3,2);

  std::cout << "\n" << _r << "\n" << std::endl;
  std::cout << "\n" << t << "\n" << std::endl;
  // write out pose in world frame
}

