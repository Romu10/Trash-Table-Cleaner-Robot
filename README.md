# CAFETERIA TABLE DETECTION
*Using only LiDAR Ranges* 

This project is **ROS 2 HUMBLE** based with the intentions of identify a 4 legs table, just using **RPLiDAR A1**,
the project have 2 parts.

### Simulation 

For simulation we will be using The Construct Cafeteria Simalation and RB1 robot from Robotnik. 

![image](https://github.com/Romu10/Trash-Table-Cleaner-Robot/assets/132951300/4f995120-c05f-4882-8ab8-6432c5adcb2b)

This RB-1 BASE mobile robot is designed for the development of indoor applications. The platform can carry different loads and materials or integrate other systems such as a robotic arm or torso.


### Real

For real application we will be using TurtleBot4 in The Construc Starbots Cafeteria.
![image](https://github.com/Romu10/Trash-Table-Cleaner-Robot/assets/132951300/250a16f6-ce75-42e2-b379-382fd2201a2b)


## Challenge 
As mentioned before the only available data in our system is de 2D LiDAR Scan. How can we identify a group of points as a table?
![image](https://github.com/Romu10/Trash-Table-Cleaner-Robot/assets/132951300/5933895f-5c35-4cbb-b483-83e252138f5a)


## Solution
With the next link you will be redirected to an online Google Colab Notebook where I explain to show step by step the process to make the identification.

Notebook: [Trash Table Detection Algorithm](https://colab.research.google.com/drive/1msecI_qLOeDJHHDH6euSB-9a1MsAxy5N#scrollTo=WKv8cxholv3W).

Remember this solutions can be the same for the real and simulated robot but not at all. 

## Branches

**Main** branch correspond to the simulation. In order to deploy the simulation in you computer git clone: 

`https://github.com/Romu10/Trash-Table-Docker.git` 

**Inside this container you will have a few workspaces.**

For launching the simulation:

`source /simulaion_ws/install/setup.bash` 

`ros2 launch the_construct_office_gazebo warehouse_rb1.launch.xml`

For launching the required nodes:

`cd /home/user/ros2_ws`

`source install/setup.bash`

`ros2 launch nav2_interface trash_table_pickup.launch.py`

For launching navigation

`ros2 launch path_planner_server pathplanner.launch.py`

For starting the process 

`python3 src/nav2_interface/nav2_interface/look_for_trash_table.py`
