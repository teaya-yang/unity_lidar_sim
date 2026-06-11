# Unity_Lidar_Sim
A simple simulator for simulating LiDAR and publishing the point cloud to ROS2.

## References
I used the following resources to build this simulator:

[ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)

[ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)

[ROS Unity Integration](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md)

[unity_ros_lidar_3d](https://github.com/sudhirpratapyadav/unity_ros_lidar_3d) (This is a ROS2 implementation with connection to other ROS packages we don't need)

## Setting up
What's in this Repo:
* ROS-TCP-Endpoint just has the ROS2 branch of the endpoint repo
* uity_ros_lidar is the Unity Project

For Unity:
* First, install [Unity Hub](https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux) on your computer.
* Open up Unity Hub, and install a version of Unity Editor. The project was edited using 2021.3.16f1 and later opened using version 6.3. There doesn't seem to be much compatibility issues.
* Once the installation completes, open the Unity project from repo. In Unity Hub, click "Add" --> "Add project from disk", and select the **"unity_ros_lidar"** inside this repo and click "Add Project".
* Once the project loads, in the Assets panel below, double-click on "Scenes" and "Sample Scene". On the right, you should see these objects in the project.

  <img width="1370" height="653" alt="image" src="https://github.com/user-attachments/assets/e88d4f6f-68d8-48b3-a050-99ead7190d57" />

For ROS:
* create and build a workspace with the following structure:
  ```
  ws_Unity/
    src/
      ROS-TCP-Endpoint/
      .../ (other ROS pakcages)
  ```
## Running the simulator
* Go to `ws_Unity` work space folder, and run `source install/local_setup.bash`.
* `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1`

* Click on the play button in Unity to start the sim.

* Once Unity is running, you should see blue arrows showing successful connection through the TCP connector.

  <img width="312" height="34" alt="image" src="https://github.com/user-attachments/assets/7767b66a-19bc-41cd-b579-9eaf2243ddf2" />

* Finally, visualize the lidar points by rviz. (You might want to increase the point size.)
  * Run `rviz2`.
  * On the "Displays" panel on the left, click "Add".
  * under "By topic", select "/point_cloud/PointCloud2".
  * Back to the "Display" panel, change the point cloud size to 0.05 m.


## External dynamics 
* A python code @scripts/trajectory_publisher.py is generating the trajectory of the ambulance and publishing it as a ROS2 topic `/ambulance/trajectory`.
* @unity_ros_lidar/Assets/Scripts/AmbulanceTrajectorySubscriber.cs listens to the topic and move the ambulance in unity. In unity editor, click the `Ambulance_no_damage` object and make sure the `AmbulanceTrajectorySubscriber` script is attached and enabled. 
* First run the @scripts/trajectory_publisher.py by `python3 trajectory_publisher.py`, then start the unity. You would see the ambulance is moving accordingly.


## Notes

* TODO: Currently the `point_cloud` topic seems to have the points in the world coordinate and not lidar coordinates. This still needs to be fixed.

* In the example, the ego vehicle is the game object called "Airplane". The LiDAR sensor is located at the "laser_link" object.

  <img width="350" height="49" alt="image" src="https://github.com/user-attachments/assets/346bdd72-54a0-40a4-86ea-8a030fcb2554" />

* In order for an object to be visible to the LiDAR, a mesh collidar must be added to it. Currently only the large objects such as the body of the airplane models have mesh collider enabled.

  <img width="949" height="759" alt="image" src="https://github.com/user-attachments/assets/ba5407fd-3326-42d3-a245-4ee1e9f9ac4a" />

* You can change the resolution of the LiDAR using the Point Cloud Publisher script under "ROS Publishers". Currently it is set to 1 degree.

  <img width="459" height="555" alt="image" src="https://github.com/user-attachments/assets/ca510bf6-3bc9-4888-ab52-17e5b7bcf45c" />

* I added a simple script to move any game object along a certain axis with a specified speed. The script is called "Move Along Axis" and attached to several game objects, including the ego vehicle.

  <img width="455" height="277" alt="image" src="https://github.com/user-attachments/assets/ee65dec6-9df8-4971-8531-8065778d3ed7" />

*To help create a visually appealing world, check out the free assets in [Unity Asset Store](https://assetstore.unity.com/3d). Add them to "My Assets" and in the Unity Project, go to "Windows" --> "Package Manager", then select the package and import it.
