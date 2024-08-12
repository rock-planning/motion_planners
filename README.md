# motion_planners

The motion_planner package contains different state of the art motion planners. This package is one of the main package of
the Motion planner framework. The Motion planner framework is focussed on to generate a collision free path for complex robotics system 
with higher degrees of freedom. This package contains different state of the art motion planners.
The current version supports optimization and sampling based motion planners. The following motion planners
are currently integrated:
- OMPL - [The Open Motion Planning Library](http://ompl.kavrakilab.org/)
- STOMP - [Stochastic trajectory optimization for motion planning](https://ieeexplore.ieee.org/document/5980280)
- TrajOpt - [Trajectory Optimization for Motion Planning](http://joschu.net/docs/trajopt-paper.pdf)

# Requirements 

![motion_planner_framework](/uploads/f3c99b5f66ce5888fde8757c9026116a/framework_1.png)

As shown in the above figure this package contains only the planner part, for collision detection and kinematics
it depends on the following library:
- [Robot Model Library](https://git.hb.dfki.de/dfki-planning/robot_model)
- [Kinematics Library](https://git.hb.dfki.de/dfki-planning/kinematics_library)
- [Collision Detection Library](https://git.hb.dfki.de/dfki-planning/collision_detection)

# Installation

On any Unix based operating systems, you can install this package. A simple way to install this package is to
use [Rock](https://www.rock-robotics.org/) framework. As this package is dependent only on the [base-types](https://github.com/rock-core/base-types)
of the [Rock](https://www.rock-robotics.org/) framework, it can also be build independent of the [Rock](https://www.rock-robotics.org/) framework. 

The optimization based planners: STOMP and TrajOpt will be installed along with this package. The sampling-based planner:
OMPL is not installed by default. It is up to the user, whether to use the sampling-based planner or not. This package
can alsp be build even without OMPL planner.

# Features

- Provides a collision free trajectory for complex robotic system.
- Handles self collision and collision with the environment.
    - Grasped object can be added to the planner and it will be checked for collision.
    - External object can be added to the environment.
- Easy to choose different planners. The code is written using factory design pattern.
- Currently in development:
    - Adding Pose constraint
    - Real time planning
    
# Directories
motion_planners contains the following directories:
- include - contains header c++ files
- src - contains source c++ files
- src/planners/   - contains planner factory class.
- src/planners/stomp - contains original [stomp](https://github.com/kalakris/stomp/tree/master/stomp) core library. In order to maintian
                        the originality of the the code, only the ROS dependency is removed and some minor corrections are made.
- src/planners/trajopt - contains original [trajopt](https://github.com/joschu/trajopt) core library. In order to maintian
                        the originality of the the code, only the openrave and bullet dependency is removed and some minor corrections are made.
                        The original code used openrave for simulation/visualization and uses bullet for collision. As we use 
                        [collision_detection](https://git.hb.dfki.de/dfki-planning/collision_detection) library for collision detection, the bullet library 
                        is stripped from the code.
- src/planners/src/wrappers - contain wrappers for OMPL, STOMP and TrajOpt library.

# Deployment
- A simple example code is in test folder.
  - usage: ./test_motion_planners "absolute_path_to_test_folder"
- Using Orogen component:
  - Please refer to [motion_planner](https://git.hb.dfki.de/dfki-planning/orogen-motion_planners) deployment package for how to deploy this libray.

                     
