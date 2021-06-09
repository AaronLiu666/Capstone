Mobile Robot Navigation and Control
====
This project is ready to run with all dependencies correctly installed on both remote computer and the Turtlebot/Raspberrypi. Since the ROS package hierachy might different check and change the package name in the command .

# Steps to start Capstone

Following tutorials below can succeddfully make the robot routing amoung points specified in in the code.

* in REMOTE PC
  ```
    # start ROS master

    roscore

    # connect to robot

    ssh pi@raspberrypi
  ```
* in TURTLEBOT
  ```
    # start bringup and camera

    roslaunch raspicam_node camerav2_410x308_30fps.launch

    roslaunch turtlebot3_bringup turtlebot3_robot.launch
  ```
* in REMOTE PC
  ```
    # start sound play node

    rosrun sound_play soundplay_node.py

    # enable raw image republish

    rosrun image_transport republish compressed in:=/raspicam_node/image raw out:=/raspicam_node/image

    # start ar tracker recognition

    rosrun beginner_tutorials listener.py

    roslaunch ar_track_alvar alvar.launch   
    ```

* in REMOTE PC
  ```
    # load the map and navigation

    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

    # run the robot

    rosrun beginner_tutorials test.py
  ```

If started correctly a RVIZ window should pop out like below.
![fig1](https://github.com/AaronLiu666/Capstone/blob/main/screenshots/Screenshot%20from%202021-06-09%2015-19-45.png)

Adjust the arrow to make the local map matches the map loaded at first. Then it is able to run.

* in REMOTE PC
  ```
  # run the robot

    rosrun beginner_tutorials test.py
  ``` 

![fig2](https://github.com/AaronLiu666/Capstone/blob/main/screenshots/Screenshot%20from%202021-06-08%2019-14-59.png)


# Code indicating position of points in test.py/test2.py




By changing those positions listed and corressponding dictionary elements in the code will lead to ideal point in the map.

```
    # start point is edited in the parameters
    self.locations['one']   = Pose(Point(3.48478,  1.91937, 0.000), Quaternion(0.000, 0.000,1, 0.00))# check
    self.locations['two']   = Pose(Point(0.4459,  2.6991, 0.000), Quaternion(0.000, 0.000, -1, 0.0536))# check
    self.locations['three'] = Pose(Point(2.1629,  2.5802, 0.000), Quaternion(0.000, 0.000, -0.05, 0.98))# check
    self.locations['four']  = Pose(Point(0.3174, -1.44786, 0.000), Quaternion(0.000, 0.000, 0.9999, 0.00001))# check
    self.locations['five']  = Pose(Point(0.29322, -0.58135, 0.000), Quaternion(0.000, 0.000, 0.016, 0.99974))# check
    self.locations['six']   = Pose(Point(4.3621, -1.4576, 0.000), Quaternion(0.000, 0.000, -0.3147, 0.949))# check
```

    


# All terminal command used

```
#PC:

roscore

rosrun sound_play soundplay_node.py

rosrun image_transport republish compressed in:=/raspicam_node/image raw out:=/raspicam_node/image

rosrun beginner_tutorials listener.py

roslaunch ar_track_alvar alvar.launch

#Pi:

roslaunch raspicam_node camerav2_410x308_30fps.launch

roslaunch turtlebot3_bringup turtlebot3_robot.launch

#navigation

roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

rosrun beginner_tutorials test.py
```





