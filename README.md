# ROS package that relies on differential drive

### Intended for learning ROS / ros_control

## Package structure
```
├── CMakeLists.txt
├── config
│   ├── diff_drive_cfg.yaml
│   ├── DynRecPID.cfg
│   ├── rqt_gui_main.perspective
│   └── rvizCfg.rviz
├── docs
│   └── doxygen
│       └── DOXY_CFG
├── launch
│   ├── gazebo.launch
│   ├── m2xr_all.launch
│   ├── m2xr_control.launch
│   ├── m2xr_world.launch
│   ├── rqt_graphs.launch
│   ├── rviz.launch
│   └── spawn.launch
├── msg
│   └── RotDbgOut.msg
├── package.xml
├── README.md
├── scripts
│   ├── config.py
│   ├── debug.py
│   ├── fsm.py
│   ├── move_to_point.py
│   └── regulator.py
├── src
├── urdf
│   ├── gazebo_macros.gazebo
│   ├── m2xr.xacro
│   ├── macros.xacro
│   ├── materials.xacro
│   ├── wheel_gazebo.xacro
│   └── wheel_transmission.xacro
└── worlds
    └── m2xr.world
```
## Launch commands

Launch gazebo world including robot model:
```
roslaunch ros_diff_drive m2xr_world.launch
```

Spawn controllers:
```
roslaunch ros_diff_drive m2xr_control.launch
```

or launch both togeher with rqt_gui:
```
roslaunch ros_diff_drive m2xr_all.launch
```

## Refered to sources:

* [[Exploring ROS using a 2 Wheeled Robot] #1: Basics of Robot Modeling using URDF](https://www.youtube.com/watch?v=jmCR225ORs0)

* [ROS Control (Gazebo tutorials)](http://gazebosim.org/tutorials/?tut=ros_control)

* [pal-robotics's pmb2_robot Github repository](https://github.com/pal-robotics/pmb2_robot)

* [[ROS Q&A] 053 - How to Move a Robot to a Certain Point Using Twist](https://www.youtube.com/watch?v=eJ4QPrYqMlw)

... and other tutorials found on the internet  

### Diff drive controller debug resources  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[Failing to launch](https://answers.ros.org/question/357979/diff_drive_controller-failing-to-launch/)
