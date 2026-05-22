# Unity_Lidar_Sim
A simple simulator for simulating LiDAR and publishing the point cloud to ros1.

## References
I used the following resources to build this simulator:

[ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)

[ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)

[ROS Unity Integration](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md)

[unity_ros_lidar_3d](https://github.com/sudhirpratapyadav/unity_ros_lidar_3d) (This is a ROS2 implementation with connection to other ROS packages we don't need)

## Setting up
For Unity:
* First, install [Unity Hub](https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux) on your computer.
* Open up Unity Hub, but don't install the recommended Unity Editor. The default project was created using 2021.3.16f1. Go to Install Editor and choose the one that starts with "2021.3" to avoid too much compatibility issue. (Any version after 2020 should in theory work, but this has not been tested)
* Once the installation completes, open the Unity project from repo. In Unity Hub, click "Add" --> "Add project from disk", and select the **"unity_ros_lidar"** inside this repo and click "Add Project".
* Once the project loads, in the Assets panel below, double-click on "Scenes" and "Sample Scene". On the right, you should see these objects in the project.
  <img width="1370" height="653" alt="image" src="https://github.com/user-attachments/assets/e88d4f6f-68d8-48b3-a050-99ead7190d57" />

For ROS:
* create and build a catkin workspace with the following structure:
  ```
  catkin_ws/
    src/
      ROS-TCP-Endpoint/
      .../ (other ROS pakcages)
    view_lidar.rviz
  ```
## Running the simulator

* `roslaunch ros_tcp_endpoint endpoint.launch`

* Click on the play button in Unity to start the sim.

* Once Unity is running, you should see blue arrows showing successful connection through the TCP connector.
  <img width="312" height="34" alt="image" src="https://github.com/user-attachments/assets/7767b66a-19bc-41cd-b579-9eaf2243ddf2" />

* Finally, visualize the lidar points by running `rviz -d view_lidar.rviz`.

## Notes

* In the example, the ego vehicle is the game object called "Airplane". The LiDAR sensor is located at the "laser_link" object.
  <img width="350" height="49" alt="image" src="https://github.com/user-attachments/assets/346bdd72-54a0-40a4-86ea-8a030fcb2554" />

* In order for an object to be visible to the LiDAR, a mesh collidar must be added to it. Currently only the large objects such as the body of the airplane models have mesh collider enabled.
  <img width="949" height="759" alt="image" src="https://github.com/user-attachments/assets/ba5407fd-3326-42d3-a245-4ee1e9f9ac4a" />

* You can change the resolution of the LiDAR using the Point Cloud Publisher script under "ROS Publishers". Currently it is set to 1 degree.
  <img width="459" height="555" alt="image" src="https://github.com/user-attachments/assets/ca510bf6-3bc9-4888-ab52-17e5b7bcf45c" />

* I added a simple script to move any game object along a certain axis with a specified speed. The script is called "Move Along Axis" and attached to several game objects, including the ego vehicle.

  <img width="455" height="277" alt="image" src="https://github.com/user-attachments/assets/ee65dec6-9df8-4971-8531-8065778d3ed7" />

*To help create a visually appealing world, check out the free assets in [Unity Asset Store](https://assetstore.unity.com/3d). Add them to "My Assets" and in the Unity Project, go to "Windows" --> "Package Manager", then select the package and import it.
