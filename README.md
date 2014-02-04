JacoROS
=======

This is a catkinized hydro compatible version of SankarNatarajan's JacoROS packag with a MoveIt! config needed to run the Kinova Jaco arm with MoveIt!

Its possible to control the arm via RViz and move_group api. The gripper controller will be added in the next commit. The API wrapper/driver still needs some improvement. E.g. to show the correct arm and finger posture in rviz the arm needs to be moved once with the joystick. Additionally sometimes the driver needs to be restarted because the arm doesn't respond anymore. Also ikFast should be used. 

#### Installation:

```sudo apt-get install ros-groovy-moveit-full mono-devel```

##### Create a catkin workspace

TODO HOWTO

##### Clone the workspaces from gitlab:
```
cd <catkin_ws>/src
wstool init
wstool set JacoROS --git https://github.com/fivef/JacoROS.git
wstool update JacoROS


##### Create workspaces
```
cd <catkin_ws>
catkin_make
source devel/setup.bash
```

Add this to your .bashrc for automatically sourcing the workspaces each time you open a terminal (replace <your_catkin_workspace>):
  source <your_catkin_workspace>/devel/setup.bash

##### For Jaco USB access:

####### Copy Jaco udev rule to your udev rules folder:
cd to the cloned "JacoROS" folder
sudo cp udev/99-jaco-arm.rules /lib/udev/rules.d/
  
####### Copy the Kinova folder from the jaco_ros package to you home folder or alternatively install the Jaco driver as described in the Jaco documentation (docs and bins can be found in the "Jaco Software" folder):
sudo cp -r Kinova ~
 
##### start components with:

```
roslaunch jaco start.launch
roslaunch jaco_moveit_config bringup_moveit_planning_execution.launch
```


The package has been cleaned up (deleted arm_navigation stuff)

Gazebo has not been testet yet.


The old documentation:

**Package Description**

1. jaco_driver <br />
  This ROS stack concerns about the jaco arm control. This stack has the following packages<br />
  - CSharpWrapper <br />
  This is not a ROS package. This C# wrapper communicates with other dlls provided by Kinova. It is thought that it will be easier to write one dll which can communicates with all other  dlls(provided by the kinova). Which in turn makes it easier for C++ wrapper. In order to communicate with jaco through C++, one needs to use this wrapper 
  - jaco <br />
  This package starts and communicates with jaco arm. It will start the jaco node, which in turn update some functionalities
      1. jaco joint publisher - reads the joint angles from the jaco and publish it.
      2. jaco action controller - its uses ROS action lib to moves jaco in joint space or in cartesian space. It also uses action lib for opening or closing fingers.
      3. jaco      - its a mono wrapper for communicating between C++ and C\#Wrapper dll.
  - jaco\_description <br />
This package contains jaco arm description. i.e urdf file and also xacro file for simulation('gazebo')
  - jaco\_arm\_kinematics <br />
  This package contains kinematics plugin for jaco arm. It uses openrave for inverse kinematic and KDL for forward solution.
  - jaco\_arm\_navigation <br />
 This package does the  motion planning for the jaco arm. It will take care of self-collision.
  - jaco\_smach 
  Ros smach used to pick and place task for jaco arm and other simple stuff. 
  
2. jaco_gazebo <br />
Its a ROS stack concerning the jaco simulation(gazebo)
  - gripper\_gazebo <br />
  Its a action lib for opening and closing gripper in gazebo
	- jaco\_gazebo\_controller\_configuration <br />
  This package has the controller parameters for jaco arm and jaco gripper. 
  

**How to use these packages in Real Jaco arm**

- roslaunch jaco start.launch <br />
This launch file intialise the jaco arm api. It will also start publishing the joint angle and starts the action lib server for moving the jaco.
- roslaunch jaco\_arm\_navigation jaco\_arm\_navigation.launch <br />
This will launch jaco arm navigation package. Kinematics, motion planning the trajectory generation are done here.
- rosrun jaco\_arm\_navigation move\_jaco\_joint\_goal<br />
simple example for moving the jaco joint.

**How to use these packages in Simulation**

One of the main requirement while developing the simulation is to write programs which can be used in both real jaco and simulation jaco.
The test code in jaco\_arm\_navigation package can also be used in Simulation. 

- roslaunch jaco\_description jaco\_gazebo.launch<br />
This launch file will start the gazebo with empty world then it will load jaco urdf file. Then it will launch a trajectory controller for jaco arm and its gripper. Finally a controller manager will be launched, currently we are using pr2 controller manager.
- roslaunch jaco\_arm\_navigation jaco\_gazebo\_navigation.launch<br />
Here jaco\_gazebo\_navigation.launch is used instead of jaco\_arm\_navigation.launch, because of different action lib messages for controlling the arm.
- rosrun jaco\_arm\_navigation move\_jaco\_joint\_goal <br />
simple example for moving the jaco joint.
- rosrun gripper\_gazebo close\_gripper / open\_gripper <br />
simple example for closing or opening the jaco gripper.
