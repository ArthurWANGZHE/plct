使用`https://eulermaker.compass-ci.openeuler.openatom.cn/api/ems1/repositories/ROS-SIG-Multi-Version_ros-humble_openEuler-24.03-LTS-TEST4/openEuler%3A24.03-LTS/x86_64/

测试日期2024/7/16

```
bash -c 'cat << EOF > /etc/yum.repos.d/ROS.repo
[openEulerROS-humble]
name=openEulerROS-humble
baseurl=https://eulermaker.compass-ci.openeuler.openatom.cn/api/ems1/repositories/ROS-SIG-Multi-Version_ros-humble_openEuler-24.03-LTS-TEST4/openEuler%3A24.03-LTS/x86_64/
enabled=1
gpgcheck=0
EOF'
```

```
dnf install "ros-humble-*" --exclude=ros-humble-generate-parameter-library-example
```




| Problem Number | Package Name                                                    | Reason for Failure                                                                                                                                                                                                                                                           |     |
| -------------- | --------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --- |
| 1              | ros-humble-mavros-2.4.0-1.oe2403.x86_64                         | 1. nothing provides GeographicLib-devel;                                                                                                                                                                                                                                     |     |
| 2              | ros-humble-irobot-create-toolbox-2.0.0-1.oe2403.x86_64          | 1. nothing provides ignition-math-devel >= 6;                                                                                                                                                                                                                                |     |
| 3              | ros-humble-mavros-extras-2.4.0-1.oe2403.x86_64                  | 1. nothing provides GeographicLib;                                                                                                                                                                                                                                           |     |
| 4              | ros-humble-moveit-planners-2.5.4-1.oe2403.x86_64                | 1. nothing provides ros-humble-moveit-planners-ompl; 2. nothing provides ros-humble-pilz-industrial-motion-planner;                                                                                                                                                          |     |
| 5              | ros-humble-plansys2-popf-plan-solver-2.0.9-1.oe2403.x86_64      | 1. nothing provides ros-humble-popf;                                                                                                                                                                                                                                         |     |
| 6              | ros-humble-ros-gz-sim-demos-0.244.9-1.oe2403.x86_64             | 1. nothing provides ros-humble-ros-gz-bridge; 2. nothing provides ros-humble-ros-gz-image; 3. nothing provides ros-humble-ros-gz-sim; 4. nothing provides ignition-gazebo5; 5. nothing provides ignition-gazebo6; 6. nothing provides ros-humble-sdformat-urdf;              |     |
| 7              | ros-humble-spacenav-3.1.0-1.oe2403.x86_64                       | 1. nothing provides libspnav-devel; 2. nothing provides spacenavd;                                                                                                                                                                                                           |     |
| 8              | ros-humble-ament-pyflakes-0.12.6-1.oe2403.x86_64                | 1. nothing provides stage-devel;                                                                                                                                                                                                                                             |     |
| 9              | ros-humble-ament-pyflakes-0.12.6-1.oe2403.x86_64                | 1. nothing provides pyflakes3;                                                                                                                                                                                                                                               |     |
| 10             | ros-humble-desktop-0.10.0-1.oe2403.x86_64                       | 1. nothing provides ros-humble-rqt-common-plugins;                                                                                                                                                                                                                           |     |
| 11             | ros-humble-gazebo-ros-pkgs-3.7.0-1.oe2403.x86_64                | 1. nothing provides ros-humble-gazebo-dev; 2. nothing provides ros-humble-gazebo-plugins; 3. nothing provides ros-humble-gazebo-ros                                                                                                                                          |     |
| 12             | ros-humble-irobot-create-common-bringup-2.0.0-1.oe2403.x86_64   | 1. nothing provides ros-humble-irobot-create-nodes;                                                                                                                                                                                                                          |     |
| 13             | ros-humble-irobot-create-ignition-bringup-2.0.0-1.oe2403.x86_64 | 1. nothing provides ros-humble-ros-ign-bridge; 2. nothing provides ros-humble-ros-ign-gazebo; 3.nothing provides ros-humble-irobot-create-ignition-plugins; 4. nothing provides ros-humble-irobot-create-ignition-toolbox; 5. nothing provides ros-humble-irobot-create-msgs; |     |
| 14             | ros-humble-moveit-ros-2.5.4-1.oe2403.x86_64                     | 1. nothing provides ros-humble-moveit-ros-visualization; 2. nothing provides ros-humble-moveit-ros-planning-interface; 3. nothing provides ros-humble-moveit-ros-benchmarks;                                                                                                 |     |
| 15             | ros-humble-pmb2-2dnav-4.0.3-1.oe2403.x86_64                     | 1. nothing provides pmb2_laser_sensors; 2. nothing provides pal_navigation_cfg_bringup;                                                                                                                                                                                      |     |
| 16             | ros-humble-ros-gz-0.244.9-1.oe2403.x86_64                       | 1. nothing provides ros-humble-ros-gz-bridge; 2. nothing provides ros-humble-ros-gz-image; 3. nothing provides ros-humble-ros-gz-sim;                                                                                                                                        |     |
| 17             | ros-humble-ros2controlcli-2.25.2-1.oe2403.x86_64                | 1. nothing provides python3-pygraphviz;                                                                                                                                                                                                                                      |     |
| 18             | ros-humble-simulation-0.10.0-1.oe2403.x86_64                    | 1. nothing provides ros-humble-ros-ign-bridge; 2. nothing provides ros-humble-ros-ign-gazebo; 3. nothing provides ros-humble-ros-ign-image;                                                                                                                                  |     |
| 19             | ros-humble-tiago-bringup-4.0.6-1.oe2403.x86_64                  | 1. nothing provides play_motion2;                                                                                                                                                                                                                                            |     |
| 20             | ros-humble-turtlebot3-cartographer-2.1.5-1.oe2403.x86_64        | 1. nothing provides ros-humble-cartographer-ros;                                                                                                                                                                                                                             |     |
| 21             | ros-humble-ur-moveit-config-2.2.6-1.oe2403.x86_64               | 1. nothing provides ros-humble-moveit-planners-ompl; 2. nothing provides ros-humble-moveit-ros-visualization; 3. nothing provides ros-humble-ur-description; 4. nothing provides ros-humble-moveit-servo;                                                                    |     |
| 22             | ros-humble-depthai-ros-2.7.1-1.oe2403.x86_64                    | 1. nothing provides depthai_descriptions; 2. nothing provides ros-humble-depthai; 3. nothing provides ros-humble-depthai-bridge; 4. nothing provides ros-humble-depthai-examples; 5. nothing provides ros-humble-depthai-ros-driver;                                         |     |
| 23             | ros-humble-dolly-0.4.0-1.oe2403.x86_64                          | 1. nothing provides ros-humble-dolly-ignition;                                                                                                                                                                                                                               |     |
| 24             | ros-humble-irobot-create-gazebo-sim-2.0.0-1.oe2403.x86_64       | 1. nothing provides ros-humble-irobot-create-gazebo-bringup; 2. nothing provides ros-humble-irobot-create-gazebo-plugins;                                                                                                                                                    |     |
| 25             | ros-humble-irobot-create-ignition-sim-2.0.0-1.oe2403.x86_64     | 1. nothing provides ros-humble-irobot-create-ignition-plugins; 2. nothing provides ros-humble-irobot-create-ignition-toolbox;                                                                                                                                                |     |
| 26             | ros-humble-mouse-teleop-1.3.0-1.oe2403.x86_64                   | 1. nothing provides python3-tk;                                                                                                                                                                                                                                              |     |
| 27             | ros-humble-moveit-2.5.4-1.oe2403.x86_64                         | 1. nothing provides ros-humble-moveit-setup-assistant;                                                                                                                                                                                                                       |     |
| 28             | ros-humble-moveit-runtime-2.5.4-1.oe2403.x86_64                 | 1. nothing provides ros-humble-moveit-ros-planning-interface; 2. nothing provides ros-humble-moveit-ros-perception;                                                                                                                                                          |     |
| 29             | ros-humble-pmb2-navigation-4.0.3-1.oe2403.x86_64                | 1. nothing provides pmb2_laser_sensors;                                                                                                                                                                                                                                      |     |
| 30             | ros-humble-rmf-building-map-tools-1.6.0-1.oe2403.x86_64         | 1. nothing provides ignition-fuel-tools7; 2. nothing provides python3-fiona; 3. nothing provides python3-pyproj; 4. nothing provides python3-rtree; 5. nothing provides python3-shapely;                                                                                     |     |
| 31             | ros-humble-rmf-visualization-2.0.0-1.oe2403.x86_64              | 1. nothing provides ros-humble-rmf-visualization-navgraphs; 2. nothing provides ros-humble-rmf-visualization-rviz2-plugins; 3. nothing provides ros-humble-rmf-visualization-schedule;                                                                                       |     |
| 32             | ros-humble-ros-ign-0.244.9-1.oe2403.x86_64                      | 1. nothing provides ros-humble-ros-ign-bridge; 2. nothing provides ros-humble-ros-ign-gazebo; 3. nothing provides ros-humble-ros-ign-image;                                                                                                                                  |     |
| 33             | ros-humble-rosbridge-suite-1.3.1-1.oe2403.x86_64                | 1. nothing provides ros-humble-rosbridge-server;                                                                                                                                                                                                                             |     |
| 34             | ros-humble-tiago-moveit-config-3.0.1-1.oe2403.x86_64            | 1. nothing provides ros-humble-moveit-planners-ompl; 2. nothing provides ros-humble-moveit-ros-visualization;                                                                                                                                                                |     |
| 35             | ros-humble-tracetools-trace-4.1.1-1.oe2403.x86_64               | 1. nothing provides python3-lttng;                                                                                                                                                                                                                                           |     |
| 36             | ros-humble-turtlebot3-simulations-2.2.5-1.oe2403.x86_64         | 1.nothing provides ros-humble-turtlebot3-gazebo;                                                                                                                                                                                                                             |     |
| 37             | ros-humble-ur-2.2.6-1.oe2403.x86_64                             | 1. nothing provides ros-humble-ur-calibration; 2. nothing provides ros-humble-ur-robot-driver;                                                                                                                                                                               |     |
| 38             | ros-humble-ur-bringup-2.2.6-1.oe2403.x86_64                     | 1. nothing provides ros-humble-ur-description;                                                                                                                                                                                                                               |     |
| 39             | ros-humble-velodyne-simulator-2.0.3-1.oe2403.x86_64             | 1. nothing provides ros-humble-velodyne-gazebo-plugins;                                                                                                                                                                                                                      |     |
| 40             | ros-humble-turtlebot3-2.1.5-1.oe2403.x86_64                     | 1. nothing provides ros-humble-cartographer-ros;                                                                                                                                                                                                                             |     |
| 41             | ros-humble-tiago-bringup-4.0.6-1.oe2403.x86_64                  | 1. nothing provides play_motion2;                                                                                                                                                                                                                                            |     |
| 42             | ros-humble-simulation-0.10.0-1.oe2403.x86_64                    | 1. nothing provides ros-humble-ros-ign-bridge; 2. nothing provides ros-humble-ros-ign-gazebo; 3. nothing provides ros-humble-ros-ign-image;                                                                                                                                  |     |
| 43             | ros-humble-ros2controlcli-2.25.2-1.oe2403.x86_64                | 1. nothing provides python3-pygraphviz;                                                                                                                                                                                                                                      |     |
| 44             | ros-humble-ros-gz-sim-demos-0.244.9-1.oe2403.x86_64             | 1. nothing provides ros-humble-ros-gz-bridge; 2. nothing provides ros-humble-ros-gz-image; 3. nothing provides ros-humble-ros-gz-sim; 4. nothing provides ignition-gazebo5; 5. nothing provides ignition-gazebo6; 6. nothing provides ros-humble-sdformat-urdf;              |     |
| 45             | ros-humble-gazebo-ros-pkgs-3.7.0-1.oe2403.x86_64                | 1. nothing provides ros-humble-gazebo-dev; 2. nothing provides ros-humble-gazebo-plugins; 3. nothing provides ros-humble-gazebo-ros;                                                                                                                                         |     |
| 46             | ros-humble-ament-pyflakes-0.12.6-1.oe2403.x86_64                | 1. nothing provides pyflakes3;                                                                                                                                                                                                                                               |     |
|                |                                                                 |                                                                                                                                                                                                                                                                              |     |



```
[root@192 Desktop]# dnf install "ros-humble-*" --exclude=ros-humble-generate-parameter-library-example

Last metadata expiration check: 0:05:07 ago on Mon 15 Jul 2024 10:41:00 PM CST.

Error:

 Problem 1: conflicting requests

  - nothing provides GeographicLib-devel needed by ros-humble-mavros-2.4.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 2: conflicting requests

  - nothing provides ignition-math-devel >= 6 needed by ros-humble-irobot-create-toolbox-2.0.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 3: conflicting requests

  - nothing provides GeographicLib needed by ros-humble-mavros-extras-2.4.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 4: conflicting requests

  - nothing provides ros-humble-moveit-planners-ompl needed by ros-humble-moveit-planners-2.5.4-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-pilz-industrial-motion-planner needed by ros-humble-moveit-planners-2.5.4-1.oe2403.x86_64 from openEulerROS-humble

 Problem 5: conflicting requests

  - nothing provides ros-humble-popf needed by ros-humble-plansys2-popf-plan-solver-2.0.9-1.oe2403.x86_64 from openEulerROS-humble

 Problem 6: conflicting requests

  - nothing provides ros-humble-ros-gz-bridge needed by ros-humble-ros-gz-sim-demos-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ros-gz-image needed by ros-humble-ros-gz-sim-demos-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ros-gz-sim needed by ros-humble-ros-gz-sim-demos-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ignition-gazebo5 needed by ros-humble-ros-gz-sim-demos-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ignition-gazebo6 needed by ros-humble-ros-gz-sim-demos-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-sdformat-urdf needed by ros-humble-ros-gz-sim-demos-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

 Problem 7: conflicting requests

  - nothing provides libspnav-devel needed by ros-humble-spacenav-3.1.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides spacenavd needed by ros-humble-spacenav-3.1.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 8: conflicting requests

  - nothing provides stage-devel needed by ros-humble-stage-ros-1.8.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 9: conflicting requests

  - nothing provides pyflakes3 needed by ros-humble-ament-pyflakes-0.12.6-1.oe2403.x86_64 from openEulerROS-humble

 Problem 10: conflicting requests

  - nothing provides ros-humble-rqt-common-plugins needed by ros-humble-desktop-0.10.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 11: conflicting requests

  - nothing provides ros-humble-gazebo-dev needed by ros-humble-gazebo-ros-pkgs-3.7.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-gazebo-plugins needed by ros-humble-gazebo-ros-pkgs-3.7.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-gazebo-ros needed by ros-humble-gazebo-ros-pkgs-3.7.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 12: conflicting requests

  - nothing provides ros-humble-irobot-create-nodes needed by ros-humble-irobot-create-common-bringup-2.0.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 13: conflicting requests

  - nothing provides ros-humble-ros-ign-bridge needed by ros-humble-irobot-create-ignition-bringup-2.0.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ros-ign-gazebo needed by ros-humble-irobot-create-ignition-bringup-2.0.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-irobot-create-ignition-plugins needed by ros-humble-irobot-create-ignition-bringup-2.0.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-irobot-create-ignition-toolbox needed by ros-humble-irobot-create-ignition-bringup-2.0.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-irobot-create-msgs needed by ros-humble-irobot-create-ignition-bringup-2.0.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 14: conflicting requests

  - nothing provides ros-humble-moveit-ros-visualization needed by ros-humble-moveit-ros-2.5.4-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-moveit-ros-planning-interface needed by ros-humble-moveit-ros-2.5.4-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-moveit-ros-benchmarks needed by ros-humble-moveit-ros-2.5.4-1.oe2403.x86_64 from openEulerROS-humble

 Problem 15: conflicting requests

  - nothing provides pmb2_laser_sensors needed by ros-humble-pmb2-2dnav-4.0.3-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides pal_navigation_cfg_bringup needed by ros-humble-pmb2-2dnav-4.0.3-1.oe2403.x86_64 from openEulerROS-humble

 Problem 16: conflicting requests

  - nothing provides ros-humble-ros-gz-bridge needed by ros-humble-ros-gz-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ros-gz-image needed by ros-humble-ros-gz-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ros-gz-sim needed by ros-humble-ros-gz-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

 Problem 17: conflicting requests

  - nothing provides python3-pygraphviz needed by ros-humble-ros2controlcli-2.25.2-1.oe2403.x86_64 from openEulerROS-humble

 Problem 18: conflicting requests

  - nothing provides ros-humble-ros-ign-bridge needed by ros-humble-simulation-0.10.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ros-ign-gazebo needed by ros-humble-simulation-0.10.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ros-ign-image needed by ros-humble-simulation-0.10.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 19: conflicting requests

  - nothing provides play_motion2 needed by ros-humble-tiago-bringup-4.0.6-1.oe2403.x86_64 from openEulerROS-humble

 Problem 20: conflicting requests

  - nothing provides ros-humble-cartographer-ros needed by ros-humble-turtlebot3-cartographer-2.1.5-1.oe2403.x86_64 from openEulerROS-humble

 Problem 21: conflicting requests

  - nothing provides ros-humble-moveit-planners-ompl needed by ros-humble-ur-moveit-config-2.2.6-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-moveit-ros-visualization needed by ros-humble-ur-moveit-config-2.2.6-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ur-description needed by ros-humble-ur-moveit-config-2.2.6-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-moveit-servo needed by ros-humble-ur-moveit-config-2.2.6-1.oe2403.x86_64 from openEulerROS-humble

 Problem 22: conflicting requests

  - nothing provides depthai_descriptions needed by ros-humble-depthai-ros-2.7.1-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-depthai needed by ros-humble-depthai-ros-2.7.1-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-depthai-bridge needed by ros-humble-depthai-ros-2.7.1-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-depthai-examples needed by ros-humble-depthai-ros-2.7.1-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-depthai-ros-driver needed by ros-humble-depthai-ros-2.7.1-1.oe2403.x86_64 from openEulerROS-humble

 Problem 23: conflicting requests

  - nothing provides ros-humble-dolly-ignition needed by ros-humble-dolly-0.4.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 24: conflicting requests

  - nothing provides ros-humble-irobot-create-gazebo-bringup needed by ros-humble-irobot-create-gazebo-sim-2.0.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-irobot-create-gazebo-plugins needed by ros-humble-irobot-create-gazebo-sim-2.0.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 25: conflicting requests

  - nothing provides ros-humble-irobot-create-ignition-plugins needed by ros-humble-irobot-create-ignition-sim-2.0.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-irobot-create-ignition-toolbox needed by ros-humble-irobot-create-ignition-sim-2.0.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 26: conflicting requests

  - nothing provides python3-tk needed by ros-humble-mouse-teleop-1.3.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 27: conflicting requests

  - nothing provides ros-humble-moveit-setup-assistant needed by ros-humble-moveit-2.5.4-1.oe2403.x86_64 from openEulerROS-humble

 Problem 28: conflicting requests

  - nothing provides ros-humble-moveit-ros-planning-interface needed by ros-humble-moveit-runtime-2.5.4-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-moveit-ros-perception needed by ros-humble-moveit-runtime-2.5.4-1.oe2403.x86_64 from openEulerROS-humble

 Problem 29: conflicting requests

  - nothing provides pmb2_laser_sensors needed by ros-humble-pmb2-navigation-4.0.3-1.oe2403.x86_64 from openEulerROS-humble

 Problem 30: conflicting requests

  - nothing provides ignition-fuel-tools7 needed by ros-humble-rmf-building-map-tools-1.6.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides python3-fiona needed by ros-humble-rmf-building-map-tools-1.6.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides python3-pyproj needed by ros-humble-rmf-building-map-tools-1.6.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides python3-rtree needed by ros-humble-rmf-building-map-tools-1.6.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides python3-shapely needed by ros-humble-rmf-building-map-tools-1.6.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 31: conflicting requests

  - nothing provides ros-humble-rmf-visualization-navgraphs needed by ros-humble-rmf-visualization-2.0.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-rmf-visualization-rviz2-plugins needed by ros-humble-rmf-visualization-2.0.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-rmf-visualization-schedule needed by ros-humble-rmf-visualization-2.0.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 32: conflicting requests

  - nothing provides ros-humble-ros-ign-bridge needed by ros-humble-ros-ign-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ros-ign-gazebo needed by ros-humble-ros-ign-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ros-ign-image needed by ros-humble-ros-ign-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

 Problem 33: conflicting requests

  - nothing provides ros-humble-rosbridge-server needed by ros-humble-rosbridge-suite-1.3.1-1.oe2403.x86_64 from openEulerROS-humble

 Problem 34: conflicting requests

  - nothing provides ros-humble-moveit-planners-ompl needed by ros-humble-tiago-moveit-config-3.0.1-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-moveit-ros-visualization needed by ros-humble-tiago-moveit-config-3.0.1-1.oe2403.x86_64 from openEulerROS-humble

 Problem 35: conflicting requests

  - nothing provides python3-lttng needed by ros-humble-tracetools-trace-4.1.1-1.oe2403.x86_64 from openEulerROS-humble

 Problem 36: conflicting requests

  - nothing provides ros-humble-turtlebot3-gazebo needed by ros-humble-turtlebot3-simulations-2.2.5-1.oe2403.x86_64 from openEulerROS-humble

 Problem 37: conflicting requests

  - nothing provides ros-humble-ur-calibration needed by ros-humble-ur-2.2.6-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ur-robot-driver needed by ros-humble-ur-2.2.6-1.oe2403.x86_64 from openEulerROS-humble

 Problem 38: conflicting requests

  - nothing provides ros-humble-ur-description needed by ros-humble-ur-bringup-2.2.6-1.oe2403.x86_64 from openEulerROS-humble

 Problem 39: conflicting requests

  - nothing provides ros-humble-velodyne-gazebo-plugins needed by ros-humble-velodyne-simulator-2.0.3-1.oe2403.x86_64 from openEulerROS-humble

 Problem 40: package ros-humble-turtlebot3-2.1.5-1.oe2403.x86_64 from openEulerROS-humble requires ros-humble-turtlebot3-cartographer, but none of the providers can be installed

  - conflicting requests

  - nothing provides ros-humble-cartographer-ros needed by ros-humble-turtlebot3-cartographer-2.1.5-1.oe2403.x86_64 from openEulerROS-humble

 Problem 41: package ros-humble-tiago-robot-4.0.6-1.oe2403.x86_64 from openEulerROS-humble requires ros-humble-tiago-bringup, but none of the providers can be installed

  - conflicting requests

  - nothing provides play_motion2 needed by ros-humble-tiago-bringup-4.0.6-1.oe2403.x86_64 from openEulerROS-humble

 Problem 42: package ros-humble-desktop-full-0.10.0-1.oe2403.x86_64 from openEulerROS-humble requires ros-humble-simulation, but none of the providers can be installed

  - conflicting requests

  - nothing provides ros-humble-ros-ign-bridge needed by ros-humble-simulation-0.10.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ros-ign-gazebo needed by ros-humble-simulation-0.10.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ros-ign-image needed by ros-humble-simulation-0.10.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 43: package ros-humble-ros2-control-2.25.2-1.oe2403.x86_64 from openEulerROS-humble requires ros-humble-ros2controlcli, but none of the providers can be installed

  - conflicting requests

  - nothing provides python3-pygraphviz needed by ros-humble-ros2controlcli-2.25.2-1.oe2403.x86_64 from openEulerROS-humble

 Problem 44: package ros-humble-ros-ign-gazebo-demos-0.244.9-1.oe2403.x86_64 from openEulerROS-humble requires ros-humble-ros-gz-sim-demos, but none of the providers can be installed

  - conflicting requests

  - nothing provides ros-humble-ros-gz-bridge needed by ros-humble-ros-gz-sim-demos-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ros-gz-image needed by ros-humble-ros-gz-sim-demos-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-ros-gz-sim needed by ros-humble-ros-gz-sim-demos-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ignition-gazebo5 needed by ros-humble-ros-gz-sim-demos-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ignition-gazebo6 needed by ros-humble-ros-gz-sim-demos-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-sdformat-urdf needed by ros-humble-ros-gz-sim-demos-0.244.9-1.oe2403.x86_64 from openEulerROS-humble

 Problem 45: package ros-humble-dolly-gazebo-0.4.0-1.oe2403.x86_64 from openEulerROS-humble requires ros-humble-gazebo-ros-pkgs, but none of the providers can be installed

  - conflicting requests

  - nothing provides ros-humble-gazebo-dev needed by ros-humble-gazebo-ros-pkgs-3.7.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-gazebo-plugins needed by ros-humble-gazebo-ros-pkgs-3.7.0-1.oe2403.x86_64 from openEulerROS-humble

  - nothing provides ros-humble-gazebo-ros needed by ros-humble-gazebo-ros-pkgs-3.7.0-1.oe2403.x86_64 from openEulerROS-humble

 Problem 46: package ros-humble-ament-cmake-pyflakes-0.12.6-1.oe2403.x86_64 from openEulerROS-humble requires ros-humble-ament-pyflakes, but none of the providers can be installed

  - conflicting requests

  - nothing provides pyflakes3 needed by ros-humble-ament-pyflakes-0.12.6-1.oe2403.x86_64 from openEulerROS-humble

(try to add '--skip-broken' to skip uninstallable packages or '--nobest' to use not only best candidate packages)
```