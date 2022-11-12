#ifndef MAP_HPP
#define MAP_HPP

#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

#include <l3xz_mapping/pose.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>
#include <mutex>
#include <stdint.h>

class Map
{
  public:
    Map(int cells_x, int cells_y, double resolution, double preview = 10.0);
    ~Map(){};

    void setOdometry(const nav_msgs::Odometry &msg);
    void addLidar(const sensor_msgs::LaserScan &msg, int8_t coeff_block = 1,
                  int8_t coeff_unblock = -1, double max_dist = 2.5);
    cv::Mat getMat() { return _map->clone(); }
    std::shared_ptr<nav_msgs::OccupancyGrid> getMap(std::string frame_id);

  private:
    static constexpr int kUnknown = -1;
    static constexpr int kMax = 100;
    std::mutex mu;
    int _cells_x, _cells_y;
    double _resolution;
    double _preview;
    cv::Mat _map_0, _map_1;

    std::shared_ptr<cv::Mat> _map;
    int _current_map_idx;
    std::pair<int, int> _odom_cells;

    Pose _p_0, _odom, _center;

    void update_cell(int x, int y, int8_t value);
    void update_map();
};

#endif
