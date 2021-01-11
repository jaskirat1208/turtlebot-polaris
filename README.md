# Compiling the project
In order to compile this code, clone this project into your catkin workspace src directory and then run catkin_make. You might want to source devel/setup.zsh to see the new nodes added.
```
cd <your catkin ws>/src
git clone <project url>
catkin_make
source devel/setup.[bash/zsh/sh] depending on your shell
```

# Running the project
In order to run the project and see the turtle self-align, do the following:

![](extras/following_turtlebot.gif)
```
roslaunch auto-align-turtlesim turtlesim.launch
```

For simulating two turtlebots in motion, 
```
roslaunch auto-align-turtlesim multi_turtlebot.launch
```

Currently, this project only supports alignment for two turtlebots or two turtles in a simulator, however, 
we can extend this functionality to different types of bots too.
