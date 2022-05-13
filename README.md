# MCCDTM
code with ICRA'22 paper - (Fast Collision Checking for Dual-Arm Collaborative Robots Working in Close Proximity)

## Description
* Forward kinematic formula using Taylor model  
  implementation of the Taylor model, see files in ./fcl/tm

* Collision checking algorithm using Taylor model  
  see demo in ./yumi_demo/src

## Requirments
* C++ (tested on C++11)
* FCL (added to this repository)
* ROS (tested on Kinetic)
* MoiveIt! (tested on MoveIt!1)

## Demos
* BVH toolkit
  build BVH for robot

* run the demo.luanch of yumi_config_demo

* run the node in ./yumi_demo/src
