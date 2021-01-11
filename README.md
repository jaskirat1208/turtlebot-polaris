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

# Example output
![](https://github.com/jaskirat1208/turtlebot-polaris/blob/jaskirat/extras/following_turtlebot.gif)
