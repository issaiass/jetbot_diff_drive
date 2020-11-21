# Jetbot_Diff_Drive

<details open>
<summary> <b>Brief Review<b></summary>

This project includes all necessary files to load an URDF xacro model in rviz and gazebo to visualize the camera and control the differential drive robot.  

Several sensors are included in the simulation like:
- 1 x RGB Camera
- 1 x Intel Realsense Depth Camera D435
- 1 x RPLIdar Laser Scan
- 8 x Sonar
- 1 x GPS
- 1 x IMU
- 1 x Odometry (see issues at the end of the document)

All sensors excluding the camera could be enabled or disabled visually.

NOTE:  
- For the realsense you will need to make the plugin first that is included [here](https://github.com/issaiass/realsense_gazebo_plugin)
- For the IMU, GPS, Odometry and sonar you will need folder hector_gazebo_plugins from [here](https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/tree/kinetic-devel/hector_gazebo_plugins) or [here](https://github.com/issaiass/hector_gazebo_plugins)
- Several dependencies will be needed, see the package description.

The robot is a WaveShare Jetbot AI Kit and its main goal is navigation.  

Below a few image examples of the outcome.

<p align="center">
<img src = "doc/imgs/jetbot_collage.PNG?raw=true" width="45%"/>
<img src = "doc/imgs/rviz_orbit.PNG?raw=true" width="35%"/>
<img src = "doc/imgs/gazebo_world.PNG?raw=true" width="49%"/>
<img src = "doc/imgs/jetbot_pointcloud.PNG?raw=true" width="75%"/>
<img src = "doc/imgs/jetbot_realsense.PNG?raw=true" width="55%"/>
<img src = "doc/imgs/jetbot_rplidar_rviz.PNG?raw=true" width="55%" />
</p>

The project tree:

<p align="center">
<img src = "doc/imgs/tree.PNG?raw=true" width="65%"/>
</p>

This applications function as follows.
- Launches rviz in a default configuration
- Launches gazebo with the ros control plugin for motors and all other sensors plugins as default
- By default all sensors are enabled visually and always enabled by its plugin
- Sensor list includes lidar, realsense, gps, imu, odometry (pose) and sonar
- Launches the rqt_robot_steering for give the linear and angular velocity commands
- After everything launches you could control the robots with the interactive rqt plugin 

</details>

<details open>
<summary> <b>Using Jetbot Differential Drive Package<b></summary>

- Put the *ball* folder from *jetbot_diff_drive/model* folder inside *~/.gazebo/models* folder to load also the soccer ball.
- See prerequisites on the package.xml, but basically are
  - joint_state_publisher
  - urdf
  - rviz 
  - roscpp
  - gazebo_ros and others
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
    git clone https://github.com/issaiass/jetbot_diff_drive
    git clone https://github.com/issaiass/realsense_gazebo_plugin
    git clone https://github.com/issaiass/hector_gazebo_plugins
    cd ..
~~~
- Go to the root folder `~/catkin_ws` and make the folder running `catkin_make` to ensure the application compiles.
- Finally launch the application by:
~~~
    # for gazebo only and rqt controller
    # (including all sensors)
    roslaunch jetbot_diff_drive jetbot_gazebo.launch 
    # (excluding Intel Realsense D435 depth camera and RPLidar)
    roslaunch jetbot_diff_drive jetbot_gazebo.launch realsense_enable:=false lidar_enable:=false
    # or for rviz only and the controller
    # (including all sensors)
    roslaunch jetbot_diff_drive jetbot_rviz.launch
    # (excluding Intel Realsense D435 depth camera and RPLidar)
    roslaunch jetbot_diff_drive jetbot_rviz.launch realsense_enable:=false lidar_enable:=false
    # or for rviz and gazebo complete simulation
    # (including all sensors)
    roslaunch jetbot_diff_drive jetbot_rviz_gazebo.launch
    # (excluding Intel Realsense D435 depth camera and RPLidar)
    roslaunch jetbot_diff_drive jetbot_rviz_gazebo.launch realsense_enable:=false lidar_enable:=false
~~~
- Examples above are suposing you want to enable the realsense, but you could enable/disable any sensor.
- You must see that `roscore` and all configurations loading succesfully.
- When everything ends, you must see gazebo and rviz loaded and the jetbot displaying a coke can in rviz and also with the intel D435 camera in front (if you enabled it, by default is enabled and also the point cloud).  Also you will see the RPLidar too over the boards.

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

Last video update - Sonar Plugin:

[<img src= "https://img.youtube.com/vi/sNLS_3OvJwk/0.jpg" />](https://youtu.be/sNLS_3OvJwk)

Previous videos list:

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

Las video Update - Sonar Plugin:

[<img src= "https://img.youtube.com/vi/_WPTCEUSzjw/0.jpg" />](https://youtu.be/_WPTCEUSzjw)

Previous videos list:

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

- For some reason odometry plugin by p3d of libhector always read (in my case) *frame id* and *child* as *odom* the next plan is to make a simple node package to get the transformation between the *base_link* and *base_footprint* to get the transform and publish in a topic.

</details>

<details open>
<summary> <b>Future Work<b></summary>

Planning to add to this project:
- :x: Probably i will add effort controllers
- :x: Navigation capabilities
- :heavy_check_mark: Computer Vision capabilities
- :x: OpenVINO as an inference engine for future deep learning based projects

</details>

<details open>
<summary> <b>Contributiong<b></summary>

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