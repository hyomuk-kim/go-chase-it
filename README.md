# Go-Chase-It Project
The second project for **_Robotics Software Engineer_** nanodegree of _[Udacity](https://www.udacity.com)_

### Project Overview
- **_ball_chaser_** package has **_process_image_** node and **_drive_bot_** node that operates respectively  
  to perceive where the white ball is located at in the camera FoV  
  and to control the motion of the mobile robot to chase the ball.
- **_my_robot_** package models the **_muks_** world with a mobile robot under [URDF](http://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html).
- Once the white ball is moved within the FoV of the front-facing camera on the robot,  
  the mobile robot will begin to chase the white ball.
- How my _ball chaser robot_ operates:
  ![myworld](https://github.com/hyomuk-kim/go-chase-it/blob/master/models_opt.gif)

### Guide
- (Pre-requisite) install Gazebo and ROS on Linux.
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace

// clone & build
$ git clone git@github.com:hyomuk-kim/go-chase-it.git
$ catkin_make

// open gazebo with the robot
$ source devel/setup.bash
$ roslaunch my_robot world.launch

// (another terminal) start ball_chaser node for robot to chase a ball:
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```

### Structure
```
go-chase-it                     # Go-Chase-It Project 
├── ball_chaser                 # ball chaser node
│   ├── launch
|   |   └── ball_chaser.launch
|   ├── src
│   │   ├── drive_bot.cpp
│   │   └── process_image.cpp
│   ├── srv
│   |   └── DriveToTarget.srv
|   ├── CMakeLists.txt
|   └── package.xml
├── my_robot                     # robot files 
│   ├── launch
|   |   ├── robot_description.launch
|   |   └── world.launch
|   ├── meshes
│   │   └── hokuyo.dae
│   ├── urdf
|   |   ├── my_robot.gazebo
│   |   └── my_robot.xacro
|   ├── worlds
│   │   └── muks.world
|   ├── CMakeLists.txt
|   └── package.xml
├── README.md
└── models_opt.gif
```

### Additional Resources
- [Gazebo plugins in ROS](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)
- [Building a visual robot model from scratch](http://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html)
