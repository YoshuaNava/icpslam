# icpslam
A SLAM system that employs 2D and 3D LIDAR measurements


## TODO
- [X] Class for handling 6 DOF poses, with time stamp, position, rotation and covariance.
- [X] Handle robot odometry.
- [X] Handle absolute robot pose from Gazebo.
- [X] Estimate odometry using ICP on LIDAR measurements.
  - [X] Cloud skipping for coping with sensors with high output frequency.
- [X] Mapping
  - [X] Implement a class to handle the current map with an Octree.
  - [X] Localization on currently built map. (inspiration: [BLAM!](https://github.com/erik-nelson/blam))
- [ ] Pose graph optimization
  - [X] Abstract class for pose optimization.
  - [X] Visualization with ROS markers.
  - [X] Integrate g2o.
  - [X] Integrate GTSAM.
  - [X] Basic pose graph with ICP transforms. ("Smoothing")
  - [X] Integrate wheel odometry.
  - [ ] Loop closure: Create and handle connections between keyframes.
  - [ ] Re-localization: compare current cloud to keyframes stored.
- Technical improvements
  - [ ] Keep a global list of Keyframes, with attributes such as key, cloud, pose, transform.
  - [ ] Make everything thread safe.
