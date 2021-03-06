block_grasp_generator
=====================

DEPRECATED in favor of [moveit_simple_grasps](https://github.com/davetcoleman/moveit_simple_grasps)

THIS REPO WILL BE DELETED IN 2015

Generic grasp generator for grasping small blocks with MoveIt. Also contains a powerful visualization_tools library for Rviz and MoveIt!

This is research code by [Dave Coleman](http://davetcoleman.com) at the Correll Robotics Lab. 

## Install

NEW: This package now depends on [moveit_visual_tools](https://github.com/davetcoleman/moveit_visual_tools)

```
git clone git@github.com:davetcoleman/moveit_visual_tools.git
```

## Tested Robots

 - [Baxter](https://github.com/davetcoleman/baxter)
 - [ClamArm](https://github.com/davetcoleman/clam)
 - [REEM](http://wiki.ros.org/Robots/REEM)

## Build Status

[![Build Status](https://travis-ci.org/davetcoleman/block_grasp_generator.png?branch=hydro-devel)](https://travis-ci.org/davetcoleman/block_grasp_generator)

## Testing and Example Code

There are currently test scripts and examples in the [baxter_pick_place](https://github.com/davetcoleman/baxter/tree/hydro-devel/baxter_pick_place) package for using this grasp generator and in [reem_tabletop_grasping](https://github.com/pal-robotics/reem_tabletop_grasping).
