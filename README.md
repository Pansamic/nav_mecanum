# nav_mecanum

navigation mecanum car


## Compile and Build

**rf2o_laser_odometry**


If there is a include error in rf2o_laser_odometry package.
Delete line `#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>` in `rf2o_laser_odometry/include/rf2o_laser_odometry/CLaserOdometry2DNode.hpp`.