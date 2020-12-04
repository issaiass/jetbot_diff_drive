# Jetbot_Diff_Drive


For the previous version files, download this repo and inside the folder do ***git checkout cd8f47e*** 


<details open>
<summary> <b>Brief Review<b></summary>

This project includes all necessary files reproduce a simulation of the waveshare Jetbot AI Kit model in rviz and gazebo to visualize the camera, control and navigate the differential drive robot.

I based the structure of this repository using [husky](https://github.com/husky/husky), [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) and [Raymon Wijnands](https://github.com/Rayman/turtlebot3_mbf) for move_base_flex repos mainly.  Other repositories like Intel and PAL Robotics are mentioned below. 

Several sensors are included in the simulation like:
- 1 x RGB Camera
- 1 x Intel Realsense Depth Camera D435
- 1 x RPLIdar Laser Scan
- 8 x Sonar
- 1 x GPS
- 1 x IMU
- 1 x Odometry (see issues at the end of the document)

All sensors excluding the jetbot camera could be enabled or disabled visually.

NOTE:  
- For the realsense you will need to make the plugin first that is included [here](https://github.com/issaiass/realsense_gazebo_plugin)
- For the IMU, GPS, Odometry and sonar you will need folder hector_gazebo_plugins from [here](https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/tree/kinetic-devel/hector_gazebo_plugins) or [here](https://github.com/issaiass/hector_gazebo_plugins)
- Several dependencies will be needed, see the *package.xml* dependencies of each one.

The robot is a WaveShare Jetbot AI Kit and its main goal is navigation.  

Below a few image examples of the outcome.

<p align="center">
<img src = "doc/imgs/jetbot_slam.PNG?raw=true" width="65%"/>
<img src = "doc/imgs/navigation.PNG?raw=true" width="55%"/>
<img src = "doc/imgs/jetbot_collage.PNG?raw=true" width="45%"/>
<img src = "doc/imgs/rviz_orbit.PNG?raw=true" width="35%"/>
<img src = "doc/imgs/jetbot_pointcloud.PNG?raw=true" width="65%"/>
</p>

The project tree:

<p align="center">
<img src = "doc/imgs/tree.PNG?raw=true" width="65%"/>
</p>

The project is now divided in several folders and now you can easily excecute effectively each file.

</details>

<details open>
<summary> <b>Using Jetbot Differential Drive Package<b></summary>

NOTE:  By default, all sensors are visually enabled.

- Put the *ball* folder from *jetbot_diff_drive/model* folder inside *~/.gazebo/models* folder to load also the soccer ball.
- See prerequisites on the *package.xml* of each package.
- Create a ROS ros workspace and compile an empty package:
~~~
    cd ~
    mkdir -p catkin_ws/src
    cd catkin_ws
    catkin_make
~~~
- Open the `.bashrc` with nano:
~~~
    nano ~/.bashrc
~~~    
- Insert this line at the end of the `~/.bashrc` file for sourcing your workspace:
~~~
    source ~/catkin_ws/devel/setup.bash
~~~
- Clone this repo in the `~/catkin_ws/src` folder by typing:
~~~ 
    cd ~/catkin_ws/src
    git clone https://github.com/issaiass/jetbot_diff_drive --recursive
    git clone https://github.com/issaiass/realsense_gazebo_plugin
    git clone https://github.com/issaiass/hector_gazebo_plugins
    cd ..
~~~
- Go to the root folder `~/catkin_ws` and make the folder running `catkin_make` to ensure the application compiles.
- Now you can test in several ways the packages
- For just only the robot description
~~~
    # 1st terminal - mount only the robot description
    roslaunch jetbot_description description.launch
    # 2nd terminal (optional) - get the robot description
    rosparam get /robot_description
~~~
- Visualizing only the robot
- You could enable more parameters if tab is pressed
~~~
    # visualize the robot in rviz
    roslaunch jetbot_viz view_model.launch
    # example... visualize the robot in rviz and disable intel realsense
    roslaunch jetbot_viz view_model.launch realsense_enable:=false
~~~
- For just view gazebo simulation (no control)
- There are more parameters for enabling sensors
- Press tab if you want to see all parameters list
- Basic spawning of the robot
~~~
    # spawn jetbot model in gazebo in turtlebot3_world
    roslaunch jetbot_gazebo spawn_jetbot.launch
    # example... spawn jetbot model in gazebo, other world
    roslaunch jetbot_gazebo spawn_jetbot.launch world_name:=<your_world>
~~~
- For controlling the jetbot in gazebo and visualize in rviz
~~~
    # launch the jetbot to control it in gazebo and visualize in rviz simultaneously
    roslaunch jetbot_control control.launch
    # OR
    # Same as above but with multiple terminals (4 terminals to launch)
    roslaunch jetbot_gazebo spawn_jetbot.launch
    roslaunch jetbot_viz view_model.launch
    roslaunch jetbot_control jetbot_controller_manager.launch
    roslaunch jetbot_rqt_robot_steering.launch

Finally, control the robot with the rqt steering controller
~~~
- For robot navigation (it is not fine tuned at this checkpoint):
~~~
    # 1st terminal, launch gazebo
    roslaunch jetbot_gazebo spawn_jetbot.launch
    # 2nd terminal, launch navigation node (dynamic window approach or time elastic band)
    # <option> = teb or dwa
    roslaunch jetbot_navigation jetbot_navigation.launch local_planner:=<option>
    # 2nd terminal, or launch navigation node (dynamic window approach only)
    # <option> = 0 or 1, 0 = move_base 1 = move_base_flex
    # Let's say we want move_base_flex, then the argument is 1
    roslaunch jetbot_navigation jetbot_navigation.launch move_base_flex:=<option>
~~~
- For robot slam:
~~~
    # 1st terminal, launch gazebo
    roslaunch jetbot_gazebo spawn_jetbot.launch
    # 2nd terminal, launch slam node (currently gmapping only)
    roslaunch jetbot_navigation jetbot_slam.launch
    # 3rd terminal, launch a controller (option 1)
    roslaunch jetbot_control jetbot_rqt_control_steering.launch
    # 3rd terminal, launch a controller (option 2)
    rosrun jetbot_twist_keyboard teleop_twist_keyboard.py
    # 4rt terminal, save the map when finished
    rosrun map_server map_saver -f <path_and_name_of_map>
~~~

- You could enable/disable any sensor in the launch file.
- You must see that `roscore` and all configurations loading succesfully.

<p align="center">
<img src = "doc/imgs/jetbot_sonar.PNG?raw=true" width="55%"/>
<img src = "doc/imgs/jetbot_willow_garage.PNG?raw=true" width="55%"/>
<img src = "doc/imgs/jetbot_realsense_rplidar.PNG?raw=true" width="55%"/>
<img src = "doc/imgs/jetbot_rviz_realsense.PNG?raw=true" width="55%"/>
<img src = "doc/imgs/jetbot_rviz_pointcloud.PNG?raw=true" width="55%"/>
<img src = "doc/imgs/rviz_camera.PNG?raw=true" width="55%"/>
</p>

<details open>
<summary> <b>Results<b></summary>

You could see the results on this youtube video.  

<p align="center">

Last video update - Jetbot AI Kit move_base_flex and dwa planner:

[<img src= "https://img.youtube.com/vi/QO-fd8mBA7Y/0.jpg" />](https://youtu.be/QO-fd8mBA7Y)

Previous videos list:

[SLAM using gmappig](https://youtu.be/SPDjOSCkUKk)

[Navigation Stack with DWA and TEB Planners](https://youtu.be/LzdAojxavZA)

[Odometry Plugin](https://youtu.be/sNLS_3OvJwk)

[Sonar Plugin](https://youtu.be/i4P4bskNwc0)

[GPS Plugin](https://youtu.be/VDtVK-NQxZk)

[IMU Plugin](https://youtu.be/iZQGH5_-pRo)

[RPLidar](https://youtu.be/7OaHnLxGrJw)

[Realsense and PLC Demo in gazebo and rviz](https://youtu.be/gretCaS2RlM)

[ROS Controllers and Camera Plugin in gazebo and rviz](https://youtu.be/_K5SHJLf5_0)
</p>

The video only shows the application running, not the explanation of the code.

</details>

<details open>
<summary> <b>Video Explanation<b></summary>

I will try my best for making an explanatory video of the application as in this youtube video.

<p align="center">

Last video update - Explaining Jetbot AI Kit move_base_flex and dwa planner:

[<img src= "https://img.youtube.com/vi/eZiigHFUuW4/0.jpg" />](https://youtu.be/eZiigHFUuW4)

Previous videos list:

[Explaining Jetbot AI Kit gmapping SLAM](https://youtu.be/2310IhapE4I)

[Explaining Navigation Stack with DWA and TEB Planners](https://youtu.be/8x3wWBDikDQ)

[Explaining Odometry Plugin](https://youtu.be/_WPTCEUSzjw)

[Explaining Sonar Plugin](https://youtu.be/_ZegRN1EfLw)

[Explaining GPS Plugin](https://youtu.be/Y7Y2SQk2QoQ)

[Explaing IMU Plugin](https://youtu.be/Fr5B8pX5c78)

[Explaining how to solve Model Sinking, not moving or drifting](https://youtu.be/1bnEdQzf8Yw)

[Explaining RPLidar](https://youtu.be/NMVvKM-G-gk)

[Explanation Realsense and PCL in gazebo and rviz](https://youtu.be/MblT-803o7M)

[Explanation ROS COntrollers and Camera Plugin in gazebo and rviz](https://youtu.be/G1z9DSnRhpI)

</p>

</details>

<details open>
<summary> <b>Issues<b></summary>

- When the navigation stack is running, in some point the map is not aligned with the laser scan, we have to test more if is the AMCL or other map parameters related to local planning.
- URDF need some modification, if you disable the link of imu, gps will not link and will cause an error.
- Always leave to true both, *imu_enable* and *gps_enable*. I will fix that later
- Planners are not fine tunned and sometimes will cause the bot to go back and forth.
- For some reason odometry plugin by p3d of libhector always read (in my case) frame id and child as odom the next plan is to make a simple node package to get the transformation between the base_link and base_footprint to get the transform and publish in a topic.

</details>

<details open>
<summary> <b>Future Work<b></summary>

Planning to add to this project:
- :x: Probably i will add effort controllers
- :heavy_check_mark: Navigation capabilities
- :x: Navigation fine tunning
- :heavy_check_mark: Computer Vision capabilities
- :x: OpenVINO as an inference engine for future deep learning based projects

</details>

<details open>
<summary> <b>Contributing<b></summary>

Your contributions are always welcome! Please feel free to fork and modify the content but remember to finally do a pull request.

</details>

<details open>
<summary> :iphone: <b>Having Problems?<b></summary>

<p align = "center">

[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawa)
[<img src="https://img.shields.io/badge/telegram-2CA5E0?style=for-the-badge&logo=telegram&logoColor=white"/>](https://t.me/issaiass)
[<img src="https://img.shields.io/badge/instagram-%23E4405F.svg?&style=for-the-badge&logo=instagram&logoColor=white">](https://www.instagram.com/daqsyspty/)
[<img src="https://img.shields.io/badge/twitter-%231DA1F2.svg?&style=for-the-badge&logo=twitter&logoColor=white" />](https://twitter.com/daqsyspty) 
[<img src ="https://img.shields.io/badge/facebook-%233b5998.svg?&style=for-the-badge&logo=facebook&logoColor=white%22">](https://www.facebook.com/daqsyspty)
[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/tiktok-%23000000.svg?&style=for-the-badge&logo=tiktok&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/whatsapp-%23075e54.svg?&style=for-the-badge&logo=whatsapp&logoColor=white" />](https://wa.me/50766168542?text=Hello%20Rangel)
[<img src="https://img.shields.io/badge/hotmail-%23ffbb00.svg?&style=for-the-badge&logo=hotmail&logoColor=white" />](mailto:issaiass@hotmail.com)
[<img src="https://img.shields.io/badge/gmail-%23D14836.svg?&style=for-the-badge&logo=gmail&logoColor=white" />](mailto:riawalles@gmail.com)

</p

</details>

<details open>
<summary> <b>License<b></summary>
<p align = "center">
<img src= "https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/by-sa.svg" />
</p>
</details>