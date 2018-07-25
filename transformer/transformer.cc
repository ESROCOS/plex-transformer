/* User code: This file will not be overwritten by TASTE. */

#include "tools/transformer/Transformer.h"
#include "transformer.h"
#include <Eigen/Core>

typedef AcyclicTransformery<3> atf;

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

atf TF;

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
  
  robot_base atf::Frame("robot_base");
    robot_base.transformToParent = odom;
  
  camera_1 = atf::Frame("camera_1");
    camera_1.transformToParent = camera_fixture;

  // TRANSFORM TREE
  TF = atf(world);

    TF.addFrame(world);
    TF.addFrame(robot_base);
    TF.addFrame(camera_1);
}

void transformer_PI_robotPose(const asn1SccBase_samples_RigidBodyState *IN_pose)
{
    
}

void transformer_PI_relativeMarkerPose(const asn1SccBase_samples_RigidBodyState *IN_pose)
{
    /* Write your code here! */
}

