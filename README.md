# a_stars
RBE 3002 code, group A Stars

## How to run

### On the robot:

run 
```
roslaunch turtlebot_bringup minimal.launch
and
roslaunch turtlebot_navigation gmappping_demo.launch
```

### On remote computer:

open rviz with rviz/exploration.rviz configuration

in separated terminals, run:

run 
```
python src/astar/astar.py
and
python src/exploration/exploration.py
```

## Simulation

There are two launch files and some maps to simulate in Stage and Gazebo. It works as it is if it's inside a package called a_star.
Otherwise you'll need to do small adaptions

## TODO

- Improve costmap update
- Organize it better


## Final Comments

If you use this code in your project, please include it in your references