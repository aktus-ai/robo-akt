# Petridish Demo Guide

## Requirements
### Install ROS2
Follow the instructions in [Ros2 Install](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html). We want ROS2 Iron version with ubuntu Jammy.

### Install Colcon
`colcon` is the build system for ROS2. Installation [guide](https://colcon.readthedocs.io/en/released/user/installation.html).

### Workspace
This repository already has a workspace. However, if you want to make one from scratch follow these [instructions](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html). After this you should have a successful `colcon build` in your workspace. Note for sourcing overlay you need to make a [new terminal](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay). While you already have a package, [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) is the instructions for how to make a package. Note at any new ROS2 related terminal you should first source the local setup from inside your workspace: `source install/local_setup.zsh`. 

### Install Yasmin
This is the state machine [library](https://github.com/uleroboticsgroup/yasmin) we are using. At this point you should be able to build the entire project with `colcon build`. If you get warnings about an unused variable in Yasmin, you could add these lines to the file `ros2_ws/src/yasmin/yasmin_demo/CMakeLists.txt`: 
```
set_source_files_properties(
  source_files
  PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter"
)
```

### Downgrade Setuptools
If `ros2 run ` fails to find your package, check the version of setuptools. If the version is greater than 58.2.0 you have to downgrade as ROS2 is not compatible with newer versions: `pip install setuptools==58.2.0`.

### Install rosdep
```
sudo apt install python3-rosdep
rosdep update
rosdep install --from-paths src -y --ignore-src

```

### Additional requirements
```
pip3 install recordclass
```

## Install RTDE library
The project can be found [here](https://sdurobotics.gitlab.io/ur_rtde/).
### Add rtde repository

```
sudo add-apt-repository ppa:sdurobotics/ur-rtde
sudo apt-get update
sudo apt install librtde librtde-dev
```

### Install RTDE

```
pip3 install  ur_rtde
```

## How to Run
You need multiple terminals for each capability.

### URSim Docker
This step is not needed if you are running it on the robot itself.

Based on the instructions [here](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/installation/ursim_docker.html#) you just need to run the following command:
```
ros2 run ur_client_library start_ursim.sh
```
The robot visualizer will be available at `http://192.168.56.101:6080/vnc.html`. In that page connect and turn on the robot. Then go to Programs -> URCaps -> External Control -> Robot Program. Click to create a new program. It will show the robot IP. On the right window click on Graphics to be able to see the robot.

### Yasmin Viewer
This is optional and will only show you the flow of your state machine. Mostly for debugging purposes.
```
ros2 run yasmin_viewer yasmin_viewer_node
```
The state machine visualizer will be on `localhost:5000`.

### Petri Demo
First, in your terminal run local setup of your workspace
```
source install/local_setup.zsh
```
The main program can run as 
```
ros2 run petridish petri_node
```
Note if you change anything in the source code, you have to build before running: `colcon build --packages-select petridish`.

## Development
If you want to add a new package or change the package dependencies take a look at some [instructions](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

If you have a new dependency, add it to the file `src/petridish/package.xml`.

### Troubleshooting
List ROS2 packages
```
ros2 pkg list
```


## Test with CultureDetectionServer
From folder `aktus-private`: `yarn dev`.
