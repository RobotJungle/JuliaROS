# JuliaROS

Packages for communicating between Julia and ROS1 

## Getting Started (ROS1 noetic)

Create a workspace
```bash 
    mkdir -p catkin_ws/src
```

Clone this repo
```bash 
    cd catkin_ws/src && git clone https://github.com/itsahmedkhalil/JuliaROS
```

Build the packages and source the workspace
```bash 
    cd .. && catkin_ws && source devel/setup.bash
```

To use the TurtleBot launch file, add the model to your bashrc using:
```bash
    echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc && source ~/.bashrc
```

Run the gazebo launch file
```bash 
    roslaunch turtlebot3_gazebo multi_turtlebot3.launch
```

Run the julia file
```bash 
    rosrun julia_pkg publisher.jl
```

## Credit:
- The turtlebot3_description package is from: https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_description
- The turtlebot3_gazebo package is from: https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/master/turtlebot3_gazebo