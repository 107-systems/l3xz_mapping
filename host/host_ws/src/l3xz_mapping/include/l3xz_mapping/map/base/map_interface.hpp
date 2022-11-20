#ifndef MAP_INTERFACE_HPP
#define MAP_INTERFACE_HPP

#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <stdint.h>

#include <l3xz_mapping/pose.hpp>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

class MapInterface
{
public:
    explicit MapInterface(int8_t coeff_block, int8_t coeff_unblock, int cells_x, int cells_y, double resolution, double preview);
    virtual ~MapInterface() = default;
    MapInterface(const MapInterface&) = delete;
    MapInterface(MapInterface&&) = delete;
    MapInterface& operator=(const MapInterface&) = delete;
    MapInterface& operator=(MapInterface&&) = delete;

    void setOdometry(const nav_msgs::Odometry &msg);
    cv::Mat getMat() { return _map->clone(); }
    std::shared_ptr<nav_msgs::OccupancyGrid> getMap(std::string frame_id);

   protected:
    static constexpr int kUnknown = -1;
    static constexpr int kMax = 100;
    int _cells_x, _cells_y;
    int8_t _coeff_block, _coeff_unblock;
    double _resolution;
    double _preview;

    std::pair<int, int> _odom_cells;
    Pose _p_0, _odom, _center;

    void update_cell(double x, double y, int8_t value);
    void update_map();

    cv::Mat _map_0, _map_1;
 
    std::shared_ptr<cv::Mat> _map;
    int _current_map_idx;
    
    std::mutex _mu;
};
#endif
