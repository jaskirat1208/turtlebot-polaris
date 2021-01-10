# Compilation instructions
In order to compile this code, clone this project into your catkin workspace src directory and then run catkin_make. You might want to source devel/setup.zsh to see the new nodes added.
```
cd <your catkin ws>/src
git clone <project url>
catkin_make
source devel/setup.[bash/zsh/sh] depending on your shell
```

# Running the project
In order to run the project and see the turtle self-align, do the following:
```
roslaunch auto-align-turtlesim turtlesim.launch
```

In case it shows a warning [cannot call service /spawn], follow the following steps instead:
```
Terminal A:
    roscore
Terminal B:
    rosrun turtlesim turtlesim_node
    
Terminal C:
    rosrun turtlesim turtle_teleop_key

Terminal D: 
    rosservice call /spawn 1 1 90 turtle2
    rosrun auto-align-turtlesim turtlesim_align
```

There is also a launch file in the project, however, there is an issue with the services due to which a turtle cannot be spawned.
