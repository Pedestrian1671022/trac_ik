/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, 
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software 
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <stdio.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ik_tests");
  ros::NodeHandle nh("~");

  int num_samples;
  std::string chain_start, chain_end, urdf_param;
  double timeout;

  nh.param("chain_start", chain_start, std::string(""));
  nh.param("chain_end", chain_end, std::string(""));
  
  if (chain_start=="" || chain_end=="") {
    ROS_FATAL("Missing chain info in launch file");
    exit (-1);
  }

  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, "/robot_description", 0.005, 1e-5, TRAC_IK::Speed);

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);
  
  if (!valid) {
    ROS_ERROR("There was no valid KDL chain found");
    return 0;
  }

  valid = tracik_solver.getKDLLimits(ll,ul);

  if (!valid) {
    ROS_ERROR("There were no valid KDL joint limits found");
    return 0;
  }

  KDL::JntArray joint_seed(chain.getNrOfJoints()), return_joints(chain.getNrOfJoints());

  for(int joint=0; joint<chain.getNrOfJoints(); ++joint) {
            joint_seed(joint) = 0.5;
        }
  
  KDL::Frame end_effector_pose(KDL::Rotation::Quaternion(0, 0, 0, 1), KDL::Vector(0.1, 0.5, 0.1));

  KDL::Twist tolerances;

  int rc = tracik_solver.CartToJnt(joint_seed, end_effector_pose, return_joints, tolerances);
 
  for(int joint=0; joint<chain.getNrOfJoints(); ++joint) {
            printf("%f\n", return_joints(joint));
  }

  ros::Publisher publisher = nh.advertise<std_msgs::Float64MultiArray>("/joint_controller/command", 10);
  std_msgs::Float64MultiArray message;
  message.data.resize(6);
  message.data[0] = return_joints(0);
  message.data[1] = return_joints(1);
  message.data[2] = return_joints(2);
  message.data[3] = return_joints(3);
  message.data[4] = return_joints(4);
  message.data[5] = return_joints(5);
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    publisher.publish(message);
    loop_rate.sleep();
  }

  return 0;
}
