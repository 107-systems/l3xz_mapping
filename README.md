<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_mapping`
============================
[![Spell Check status](https://github.com/107-systems/l3xz_mapping/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/l3xz_mapping/actions/workflows/spell-check.yml)

This repository includes the ROS-based explorative mapping stack of L3X-Z for ELROB 2022.

The stack is tested under ROS1 Noetic Ninjemys. It can do mapping using [RTAB-Map](https://github.com/introlab/rtabmap_ros) and a [simple known-position approach](http://ais.informatik.uni-freiburg.de/teaching/ss12/robotics/slides/ss10/08-occupancy-mapping.pdf). Also data logging is possible.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

# Installation
Clone the repository and go to host root:
~~~bash
git clone --recursive https://github.com/107-systems/l3xz_mapping
cd l3xz_mapping/host_ws
~~~

Build using catkin:
~~~bash
catkin_make install
~~~

There is also a [frontend](https://github.com/107-systems/l3xz_frontend) available for the mapping stack.
## Start mapping

### RTAB-Map
~~~bash
roslaunch l3xz_mapping l3xz_mapping.launch
~~~

### Known-position approach
~~~
roslaunch l3xz_mapping knownposition_lidar.launch
~~

### Data logging
roslaunch odom_recorder.launch
roslaunch thermal_recorder.launch
roslaunch plotter_grid.launch
~~~

## Nodes

### transform_odom

Creates an odometry message from tf data.

#### Published Topics
| Default Name | Type |
|:-:|:-:|
| `/odom_slam` | [`nav_msgs/Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) |

#### Parameters
| Name | Default | Description |
|:-:|:-:|-|
| `odometry_out` | `/odom_slam` | Odometry topic name |
| `parent_frame` | `/map` | Parent frame of interesting tf tree part |
| `child_frame` | `/base_link` | Child frame of interesting tf tree part |

### plotter

Plots robot track and OPIs according to ELROB rules into a map. The input map can be an image or data from a topic of type `nav_msgs/OccupancyGrid`. The OPIs to be plotted and the robot track are subscribed from a topic of type `l3xz_mapping/Track`.

#### Parameters
| Name | Default | Description |
|:-:|:-:|-|
| `from_grid` | `false` | Get map topic from OccupancyGrid |
| `img_src` | `/home/log/map.png` | Map image name |
| `img_dest` | `/home/log/map_dest.png` | Destination map image name |
| `meters_per_pixel` | 0.1 | Resolution of map image |
| `bearing_deg` | 0.0 | Robot initial bearing |
| `zero_px` | [100, 100] | Robot initial position |
| `robot_track` | `/l3xz/odom_recorder/track` | Track topic |
| `opi_tracks` | [`/l3xz/thermal_recorder/track`, `/l3xz/radiation_recorder/track`] | OPI topics |
| `grid_topic` | `/grid` | Grid map |

### recorder

Records waypoints according to ELROB log format with Unix timestamp and WSG84 coordinates with images of the artifacts. Also a track containing all waypoints can be published (e.g. for plotting). All the logs are based on an initial startpoint.

#### Service clients

| Default Name | Purpose |
|:-:|:-:|
| `recorder/set_startpoint` | Set initial robot position. Needs to be done only once and is necessary to start recording. |
| `recorder/set_waypoint` | Record a new waypoint. |

#### Published Topipcs
| Default Name | Type |
|:-:|:-:|
| `recorder/track` | [`l3xz_mapping/Track`] |

#### Parameters
| Name | Default | Description |
|:-:|:-:|-|
| `logpath` | `/home/l3xz/log.txt` | Path for logfile |
| `artifacts` | [] | List of artifact images |
| `track_publishing_rate` | 1 | Period for publishing track rate |
| `publish_track` | `true` | Enable publishing of track |

### knownposition_node

For some tasks a complete map of an area is not desired. Therefore, it is a good idea to use a robust mapping from known position approach instead of SLAM to observe the area around the robot to master autonomous navigation tasks. This node demonstrates this approach using the data of one 2D-Lidar. All features of the implemented algorithms, e.g. overlaying multiple datasets, image postprocessing or transforming using tf are not demonstrated yet.

<p align="center">
    <img src="doc/knownposition.png">
</p>

#### Subscribed Topipcs
| Default Name | Type |
|:-:|:-:|
| `/lidar` | [`Input lidar data`] |
| `/odom` | [`Odometry data`] |

#### Published Topipcs
| Default Name | Type |
|:-:|:-:|
| `/grid` | [`Occupancy grid map`] |

#### Parameters
| Name | Default | Description |
|:-:|:-:|-|
| `pixel_x` | `600` | Columns of the gridmap |
| `pixel_y` | `600` | Rows of the gridmap |
| `resolution` | `0.1` | Resolution of the gridmap in meters |
| `rate_hz` | `5` | Period for publishing the gridmap |
| `grid_topic` | `/grid` | Name of the gridmap |
| `odometry_topic` | `/odometry` | Name of the odometry topic |
| `show` | `true` | Show map in OpenCV window (like above) |
