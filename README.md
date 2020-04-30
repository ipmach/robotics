# Robotics

Run world

```python
roslaunch assign2 thymio_gazebo_bringup.launch name:=thymio10 world:=arena
```

Run robot

```python
roslaunch assign2 controller.launch 
```

## Scripts

* part1.py: main code .
* momevement_class.py: class use to work with odometry of the robot.
* sensor_class.py: class use to work with the sensor of the robot.
* interface.py: interface of the robot (plots,graphs ect...).
* neuronalNetwork.py: empty class used to try to see how to collect data (can be remove).

momevement and sensor have all the functions and logic need it to work with sensors and direction of the robot.

## Launch 

controller.launch have parameters inside to program the use of the robot:
* robot_name: name of the robot.
* instruction_path: path of the general instruction to follow. (exercise2.json default, exercise1.json is to move in a 8).
* use_sensors: flag to tell the robot if to use sensors (Close loop or Open loop).
* random_noise: flag to use the second controller to create random moves.

The next two are in the controller.launch but not implement it (can be remove):
* use_net: to use the need to detect obstacles. 
* train_net: to train the net.

## Diagrams

Diagrams for better understanding of the code.

 * Flags use in the launch file

![Graph](https://github.com/ipmach/Robotics/blob/master/img/Flags.png)

 * States of the main controller

![Graph](https://github.com/ipmach/Robotics/blob/master/img/States_robot.png)

* Main diagram of the code

![Graph](https://github.com/ipmach/Robotics/blob/master/img/initial_diagram.png)
