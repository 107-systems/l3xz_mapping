#ifndef MAP_BASE_HPP
#define MAP_BASE_HPP

#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
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

template<class T>
class MapBase {
       public:
        explicit MapBase(int8_t coeff_block, int8_t coeff_unblock, int cells_x, int cells_y, double resolution, double preview = 10.0, int queue_size = 10);
        virtual ~MapBase();
        MapBase(const MapBase&) = delete;
        MapBase(MapBase&&) = delete;
        MapBase& operator=(const MapBase&) = delete;
        MapBase& operator=(MapBase&&) = delete;

        void setOdometry(const nav_msgs::Odometry &msg);
        void add(const T &msg);
        virtual void eval(std::shared_ptr<T> msg){};
        cv::Mat getMat() { return _map->clone(); }
        std::shared_ptr<nav_msgs::OccupancyGrid> getMap(std::string frame_id);

       protected:
        static constexpr int kUnknown = -1;
        static constexpr int kMax = 100;
        std::mutex _mu;
        std::thread _thread;
        bool _running;
        void worker();
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
        int _queue_size;
        std::queue<std::shared_ptr<T>> _messages;
};

template class MapBase<sensor_msgs::LaserScan>;
#endif
