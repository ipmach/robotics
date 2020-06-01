# Robotics

## Environment

Run world without big interface 

```python 
roslaunch robotics thymio_gazebo_bringup.launch name:=thymio10 world:=pink_hall gui:=false
```
Run world with big interface

```python
roslaunch robotics thymio_gazebo_bringup.launch name:=thymio10 world:=pink_hall gui:=true
```

## Robot controller

Run robot with saving data and debug on

```python
roslaunch robotics controller.launch dataset_size:=1000 debug:=1
```
Run robot without saving data

```python
roslaunch robotics controller.launch save_data:=0 
```

Run robot with cnn controller

```python
roslaunch robotics cnn_controller.launch save_data:=0 
```

## Build ros package

```python
catkin build
```

```python
re.
```

## Scripts

* sensor_controller.py: main code .
* momevement_class.py: class use to work with odometry of the robot.
* ros_messages.py: class with all the subcribers and publishers
* sensor_class.py: class use to work with the sensor of the robot.
* interface.py: interface of the robot (plots,graphs ect...).
* model_controller.py: empty class used to try to see how to collect data (can be remove).
* controller_dot.py: alternative logic to move the robot using the sensors.
* model_controler_alt.py: alternative controller to use the CNN model.

momevement and sensor have all the functions and logic need it to work with sensors and direction of the robot.

## Launch 

controller.launch/ cnn_controller.launch have parameters inside to program the use of the robot:
* robot_name: name of the robot.
* instruction_path: path of the general instruction to follow. (exercise2.json default, exercise1.json is to move in a 8).
* use_sensors: flag to tell the robot if to use sensors (Close loop or Open loop).
* random_noise: flag to use the second controller to create random moves.
* save_data: flag to say if the robot gather data
* data_path: path where the robot save the data
* dataset_size: number of images to save
* debug: activate debug_mode
* debug_path: path of the logs
* model: model use
* model_path: path of the folder of the model




