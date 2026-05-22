# Unity_Lidar_Sim
A simple simulator for simulating LiDAR and publishing the point cloud to ros1.

## References
I used the following resources to build this simulator:

[ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
[ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)
[ROS Unity Integration](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md)
[unity_ros_lidar_3d](https://github.com/sudhirpratapyadav/unity_ros_lidar_3d)

## Setting up
For Unity:
* First, [install Unity Hub](https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux) on your computer.
* Open up Unity Hub, but don't install the recommended Unity Editor. The default project was created using 2021.3.16f1. Go to Install Editor and choose the one that starts with "2021.3" to avoid too much compatibility issue. (Any version after 2020 should in theory work, but this has not been tested)
* Once the installation completes, open the Unity project from repo. In Unity Hub, click Add -> Add project from disk, and select the **"unity_ros_lidar"** inside this repo and click "Add Project".
* Once the project loads, in the Assets panel below, double-click on "Scenes" and "Sample Scene". On the right, you should see these objects in the project.

For ROS:
* create and build a catkin workspace with the following structure:
  ```
  catkin_ws/
    src/
      ROS-TCP-Endpoint/
      .../
    view_lidar.rviz
  ```

## Running the simulator

`roslaunch ros_tcp_endpoint endpoint.launch`

Click on the play button in Unity to start the sim.

Once Unity is running, you should see blue arrows showing successful connection through the TCP connector.

Finally, visualize the lidar points by running 'rviz -d view_lidar.rviz'

## Notes
