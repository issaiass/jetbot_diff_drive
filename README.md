# Jetbot_Diff_Drive

<details open>
<summary> <b>Brief Review<b></summary>

This project includes all necessary files to load an URDF xacro model in rviz and gazebo to visualize the camera and control the differential drive robot.  Now i included a intel realsense D435 camera and the point clouds plugin and you can disable/enable the D435 and point clouds if you wish.

The robot is a WaveShare Jetbot AI Kit and its main goal is navigation.  

Below a few image examples of the outcome.

<p align="center">
<img src = "doc/imgs/rviz_orbit.PNG?raw=true" width="35%"/>
<img src = "doc/imgs/gazebo_world.PNG?raw=true" width="49%"/>
<img src = "doc/imgs/jetbot_pointcloud.PNG?raw=true" width="75%"/>
<img src = "doc/imgs/jetbot_realsense.PNG?raw=true" width="55%"/>
</p>

The project tree:

<p align="center">
<img src = "doc/imgs/tree.PNG?raw=true" width="45%"/>
</p>

This applications function as follows.
- Launches rviz in a default configuration
- Launches gazebo with the ros control plugin for motors and the camera plugin
- This application also loads an intl realsense D435 if is enabled
- Launches the rqt_robot_steering for give the linear and angular velocity commands
- After everything launches you could control with the interactive rqt plugin 

</details>

<details open>
<summary> <b>Using Jetbot Differential Drive Package<b></summary>

- Put the *ball* folder from *jetbot_diff_drive/model* folder inside *~/.gazebo/models* folder to load also the soccer ball.
- See prerequisites on the package.xml, but basically are
  - joint_state_publisher
  - urdf
  - rviz 
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
    rm -rf README.md
    rm -rf doc/
~~~
- Go to the root folder `~/catkin_ws` and make the folder running `catkin_make` to ensure the application compiles.
- Finally launch the application by:
~~~
    # for gazebo only and rqt controller
    # (including Intel Realsense D435 depth camera)
    roslaunch jetbot_diff_drive jetbot_gazebo.launch 
    # (excluding Intel Realsense D435 depth camera)
    roslaunch jetbot_diff_drive jetbot_gazebo.launch realsense_enable:=falase     
    # or for rviz only and the controller
    # (including Intel Realsense D435 depth camera)
    roslaunch jetbot_diff_drive jetbot_rviz.launch
    # (excluding Intel Realsense D435 depth camera)
    roslaunch jetbot_diff_drive jetbot_rviz.launch realsense_enable:=false    
    # or for rviz and gazebo complete simulation
    # (including Intel Realsense D435 depth camera)
    roslaunch jetbot_diff_drive jetbot_rviz_gazebo.launch
    # (excluding Intel Realsense D435 depth camera)
    roslaunch jetbot_diff_drive jetbot_rviz_gazebo.launch realsense_enable:=false
~~~
- You must see that `roscore` and all configurations loading succesfully.
- When everything ends, you must see gazebo and rviz loaded and the jetbot displaying a coke can in rviz and also with the intel D435 camera in front (if you enabled it, by default is enabled and also the point cloud).

<p align="center">
<img src = "doc/imgs/jetbot_rviz_realsense.PNG?raw=true" width="55%"/>
<img src = "doc/imgs/jetbot_rviz_pointcloud.PNG?raw=true" width="55%"/>
<img src = "doc/imgs/rviz_camera.PNG?raw=true" width="55%"/>
</p>

<details open>
<summary> <b>Results<b></summary>

You could see the results on this youtube video.  

<p align="center">

Last video update:

[<img src= "https://img.youtube.com/vi/gretCaS2RlM/0.jpg" />](https://youtu.be/gretCaS2RlM)

Previous videos:

[<img src= "https://img.youtube.com/vi/_K5SHJLf5_0/0.jpg" />](https://youtu.be/_K5SHJLf5_0)
</p>

The video only shows the application running, not the explanation of the code.

</details>

<details open>
<summary> <b>Video Explanation<b></summary>

I will try my best for making an explanatory video of the application as in this youtube video.

<p align="center">

Las video Update:

[<img src= "https://img.youtube.com/vi/hpUCG6K5muI/0.jpg" />](https://youtu.be/hpUCG6K5muI)

Previous videos:

[<img src= "https://img.youtube.com/vi/G1z9DSnRhpI/0.jpg" />](https://youtu.be/G1z9DSnRhpI)

</p>

</details>

<details open>
<summary> <b>Issues<b></summary>

- Not an issue but you must know.  The PID was inserted via .yaml because i was not able to insert it on the URDF. 

</details>

<details open>
<summary> <b>Future Work<b></summary>

Planning to add to this project:
- Probably i will add effort controllers
- Navigation capabilities
- Computer Vision capabilities
- OpenVINO as an inference engine for future deep learning based projects

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