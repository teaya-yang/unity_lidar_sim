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
* create a symlink like this:
  `ln -s ./Unity_Lidar_Sim/ROS-TCP-Endpoint ./ws_Unity/src/ROS-TCP-Endpoint`

## Running the simulator
* Go to `ws_Unity` and build the package with `colcon build`.
* Run `source install/local_setup.bash`.
* To start the ros2 connection with unity `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1`

* Click on the play button in Unity to start the sim.

* Once Unity is running, you should see blue arrows showing successful connection through the TCP connector.

  <img width="312" height="34" alt="image" src="https://github.com/user-attachments/assets/7767b66a-19bc-41cd-b579-9eaf2243ddf2" />

* Finally, visualize the lidar points by rviz. (You might want to increase the point size.)
  * Run `rviz2`.
  * On the "Displays" panel on the left, click "Add".
  * under "By topic", select "/point_cloud/PointCloud2".
  * Back to the "Display" panel, change the point cloud size to 0.05 m.


## Data collection pipeline

The full pipeline records synchronized LiDAR point clouds and ground-truth agent labels across automatically generated episodes.

### Components

| Component | Role |
|---|---|
| `ScenarioConfig` assets | Define agent count, speed, spawn area per scenario |
| `EpisodeSweepRunner` | Cycles through (config, seed) pairs automatically |
| `AgentPlacementRandomizer` | Spawns agents at randomized NavMesh positions |
| `PatrolPathRandomizer` | Assigns a unique random patrol route to each agent |
| `GroundTruthPublisher` | Publishes agent positions + states to `/ground_truth/agents` |
| `ros2 bag record` | Records all topics to disk |

### Step-by-step

**Terminal 1 — ROS endpoint**
```bash
cd ws_Unity
source install/local_setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
```

**Terminal 2 — start recording before pressing Play**
```bash
# Bags are saved under Unity_Lidar_Sim/bags/ — create it once if it doesn't exist
mkdir -p ~/Unity_Lidar_Sim/bags
ros2 bag record /point_cloud /ground_truth/agents \
  -o ~/Unity_Lidar_Sim/bags/sweep_s1_$(date +%s)
```

**Unity — press Play**

The `EpisodeSweepRunner` takes over:
1. Sets the active `ScenarioConfig` on `ScenarioManager`
2. Runs all randomizers (placement → patrol paths → any others)
3. Waits `settleTime` seconds for agents to start moving
4. Records for `episodeDuration` seconds
5. Advances to next (config, seed) pair and repeats

The Console prints a line on every episode transition:
```
[SWEEP] episode 3/30 | config=S1_Dense | seed=2 | agents=30 spawnRadius=40
```
Use these timestamps to slice the bag into labelled episodes in post-processing.

**Terminal 2 — stop recording when sweep finishes**
```
Ctrl+C
```

### Output

```
~/Unity_Lidar_Sim/bags/
  sweep_s1_<timestamp>/
    metadata.yaml
    sweep_s1_<timestamp>_0.db3    ← point clouds + ground truth, all episodes

~/.config/unity3d/DefaultCompany/unity_ros_lidar_3d/sweep_log.json  ← episode index: config, seed, agent count, Unix timestamp
```

The `.db3` bag and `sweep_log.json` together give you fully labelled, reproducible episodes. Pair frames by matching the `stamp_sec/stamp_nsec` fields in the `/ground_truth/agents` JSON to the `PointCloud2` header timestamp — both use `Clock.time` as their source.

### Dataset contents

Every recorded frame contains two synchronized messages:

**`/point_cloud` — `sensor_msgs/PointCloud2`**

Raw LiDAR point cloud from the raycast sensor. Each point is 16 bytes:

| Field | Type | Description |
|---|---|---|
| `x, y, z` | float32 | Point position in sensor frame (metres) |
| `intensity` | float32 | Simulated return intensity |

Typical frame: ~10 000–50 000 points depending on FOV and angular resolution settings.

**`/ground_truth/agents` — `std_msgs/String` (JSON)**

One JSON object per frame with full scene state:

```json
{
  "stamp_sec": 42, "stamp_nsec": 100000000,
  "episode": 3, "config": "ScenarioConfig_Dense", "seed": 2,
  "ego": {
    "tx": 0.0, "ty": 0.0, "tz": 15.0,
    "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
    "yaw": 0.0
  },
  "agents": [
    {
      "id": 12345,
      "type": "Pedestrian",
      "state": "Patrolling",
      "rx": 5.231, "ry": -2.100, "rz": 0.0,
      "yaw": 1.047,
      "bbox": { "cx": 5.231, "cy": -2.100, "cz": 0.9,
                "sx": 0.6,   "sy": 0.6,   "sz": 1.8 }
    }
  ],
  "vehicles": [
    {
      "id": 67890,
      "type": "Vehicle",
      "state": "Moving",
      "rx": 12.0, "ry": -4.5, "rz": 0.0,
      "yaw": 0.52,
      "bbox": { "cx": 12.0, "cy": -4.5, "cz": 1.1,
                "sx": 4.5,  "sy": 2.0,  "sz": 2.2 }
    }
  ]
}
```

| Field | Description |
|---|---|
| `stamp_sec/nsec` | ROS sim-time timestamp — matches the PointCloud2 header for frame pairing |
| `episode/config/seed` | Which sweep episode this frame belongs to |
| `ego` | Ego sensor platform (airplane) pose — global ROS frame, nuScenes-style `ego_pose` |
| `ego.tx/ty/tz` | Ego translation, global ROS frame |
| `ego.qw/qx/qy/qz` | Ego orientation quaternion, global ROS frame |
| `id` | **Stable** per-object id (Unity `GetInstanceID`) — same object keeps its id across frames within an episode |
| `type` | `Pedestrian`, `Animal`, or `Vehicle` |
| `state` | `Patrolling`, `Wandering`, `Reacting`, `Paused`, `Crossing`, or `Moving` |
| `rx/ry/rz` | World position in ROS frame (x=forward, y=left, z=up) — object pivot |
| `yaw` | Heading in radians, ROS convention |
| `bbox.cx/cy/cz` | Bounding box **center** in ROS frame (not pivot point) |
| `bbox.sx/sy/sz` | Bounding box **full extents** in metres (x=length, y=width, z=height) |

**`sweep_log.json`** — written at `~/.config/unity3d/DefaultCompany/unity_ros_lidar_3d/sweep_log.json` at the end of the sweep. Maps each episode number to its config name, seed, agent count, and Unix timestamp — use this to slice the bag by episode in post-processing.

### Converting to nuScenes format

`scripts/bag_to_nuscenes.py` turns a recorded bag into a nuScenes-style dataset (subset
of tables: `scene`, `sample`, `sample_data`, `sample_annotation`, `instance`, `category`,
`attribute`, `visibility`, `ego_pose`, `calibrated_sensor`, `sensor`, `log`).

```bash
source /opt/ros/humble/setup.bash          # needs rclpy to deserialize the bag
python3 scripts/bag_to_nuscenes.py \
    --bag ~/Unity_Lidar_Sim/bags/sweep_s1_<timestamp> \
    --out ~/Unity_Lidar_Sim/nuscenes_out \
    --version v1.0-mini
```

Output layout:
```
nuscenes_out/
  samples/LIDAR_TOP/<episode>_<frame>.pcd.bin   ← clouds, SENSOR frame, float32 x,y,z,intensity
  v1.0-mini/*.json                               ← the metadata tables
```

Mapping and conventions:

| Recorded data | nuScenes table |
|---|---|
| one episode | `scene` |
| one GT frame | `sample` + `ego_pose` + `sample_data` |
| one agent/vehicle in a frame | `sample_annotation` |
| unique `(episode, id)` | `instance` |
| `type` | `category` |
| `state` | `attribute` |

* **Every frame is a keyframe** — labelling is free in sim, so each `sample_data` is also a `sample` (`is_key_frame=true`).
* **Frames**: `sample_annotation` + `ego_pose` are **global**; point clouds are written in the **sensor** frame so the nuScenes chain sensor→ego→global reconstructs global exactly. `calibrated_sensor` is identity (LiDAR coincident with ego — set its `translation`/`rotation` if you want the real lever-arm).
* **`size`** is reordered to nuScenes `[width, length, height]` = `[bbox.sy, bbox.sx, bbox.sz]`.
* **`num_lidar_pts`** is computed by a point-in-box test in the global frame.
* **Tokens** are deterministic MD5 of a stable key, so re-running the converter is reproducible.

> By default the converter assumes the recorded cloud is in the **global** frame (matches the
> current `point_cloud` TODO below). If you fix the sensor to emit truly sensor-local points,
> the global→sensor transform must be removed — adjust `convert()` accordingly.

### Replaying and inspecting

```bash
# list topics and message counts
ros2 bag info ~/Unity_Lidar_Sim/bags/sweep_s1_<timestamp>/

# verify both topics are present — should show /point_cloud and /ground_truth/agents
ros2 bag info ~/Unity_Lidar_Sim/bags/sweep_s1_<timestamp>/ | grep Topic

# replay at half speed for inspection in rviz2
ros2 bag play ~/Unity_Lidar_Sim/bags/sweep_s1_<timestamp>/ --rate 0.5

# print ground truth messages (while bag is playing)
ros2 topic echo /ground_truth/agents
```

---

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