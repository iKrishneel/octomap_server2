<div align="center">
  <h1>OctoMap Server2</h1>
  <h3>Implementation of octomap for ROS2.0 </h3>
    <a href="https://travis-ci.com/iKrishneel/octomap_server2"><img src="https://travis-ci.com/iKrishneel/octomap_server2.svg?branch=master"></a>
</div>

Port of the ROS1 [octomap server](https://github.com/OctoMap/octomap_mapping) for ROS2.0 

#### Installation
Firstly make sure you have [octomap](https://github.com/OctoMap/octomap.git) installed on your system 

Next, clone this ros package to the appropriate ros2 workspace
```bash
$ git clone https://github.com/iKrishneel/octomap_server2.git
```
Clone the dependency repositories to the workspace
```bash
# will clone octomap_msgs to the workspace
$ vcs import . < deps.repos
```

#### Building
Use colcon to build the workspace
```bash
$ colcon build --symlink-install --packages-select octomap_msgs octomap_server2
```

#### Running
Launch the node with appropriate input on topic `cloud_in`
```bash
$ ros2 launch octomap_server2 octomap_server_launch.py
```