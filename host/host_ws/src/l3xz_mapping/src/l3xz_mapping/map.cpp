#include <l3xz_mapping/map.hpp>

Map::Map(int cells_x, int cells_y, double resolution, double preview)
    : _cells_x(cells_x), _cells_y(cells_y), _resolution(resolution), _current_map_idx(0),
      _preview(preview)
{
    _map_0 = cv::Mat(_cells_x, _cells_y, CV_8UC1);
    _map_1 = cv::Mat(_cells_x, _cells_y, CV_8UC1);
    _map = std::make_shared<cv::Mat>(_map_0);
    for (int y = 0; y < _map->cols; y++)
    {
        for (int x = 0; x < _map->rows; x++)
        {
            _map->at<int8_t>(y, x) = kUnknown;
        }
    }

    _p_0.x = -_resolution * _cells_x * 0.5;
    _p_0.y = -_resolution * _cells_y * 0.5;
}

void Map::update_cell(double x, double y, int8_t value)
{
    Pose pivot;
    pivot.x = -_p_0.x;
    pivot.y = -_p_0.y;
    pivot.x += _odom.x;
    pivot.y += _odom.y;
    pivot.yaw = _odom.yaw;
    double s = sin(pivot.yaw - 0.5 * M_PI);
    double c = cos(pivot.yaw - 0.5 * M_PI);

    double x_m = x * c - y * s;
    double y_m = x * s + y * c;

    int xx = static_cast<int>((x_m + pivot.x) / _resolution);
    int yy = _cells_y - static_cast<int>((y_m + pivot.y) / _resolution);

    if (xx >= 0 && xx < _cells_x && yy >= 0 && yy < _cells_y)
    {
        int32_t current = static_cast<uint32_t>(_map->at<int8_t>(yy, xx));
        if (kUnknown == current)
        {
            current = kMax / 2;
        }
        if (value > 0 && current < 100)
        {
            _map->at<int8_t>(yy, xx) = static_cast<int8_t>(current + value);
        }
        else if (current > 0)
        {
            _map->at<int8_t>(yy, xx) = static_cast<int8_t>(current + value);
        }
    }
}

void Map::setOdometry(const nav_msgs::Odometry &msg)
{
    const std::lock_guard<std::mutex> lock(mu);
    _odom.x = static_cast<double>(msg.pose.pose.position.x);
    _odom.y = static_cast<double>(msg.pose.pose.position.y);
    double x = msg.pose.pose.orientation.x;
    double y = msg.pose.pose.orientation.y;
    double z = msg.pose.pose.orientation.z;
    double w = msg.pose.pose.orientation.w;
    _odom.yaw = atan2(2.0 * (w * z + w * y), w * w + x * x - y * y - z * z) + 0.5 * M_PI;
    int xx = static_cast<int>(fabs(_odom.dX(_p_0) / _resolution));
    int yy = _cells_y - static_cast<int>(fabs(_odom.dY(_p_0) / _resolution));
    _odom_cells = std::make_pair(xx, yy);
    update_map();
}

void Map::update_map()
{
    static double offset_max = std::min(_cells_x * _resolution * 0.5, _cells_y * _resolution * 0.5);
    if (offset_max < _center.dist2D(_odom) + _preview)
    {
        std::shared_ptr<cv::Mat> new_map;
        if (0 == _current_map_idx)
        {
            new_map = std::make_shared<cv::Mat>(_map_1);
        }
        else
        {
            new_map = std::make_shared<cv::Mat>(_map_0);
        }

        for (int y = 0; y < new_map->cols; y++)
        {
            for (int x = 0; x < new_map->rows; x++)
            {
                new_map->at<int8_t>(y, x) = kUnknown;
            }
        }

        int x_start = std::max(0, _odom_cells.first - _cells_x / 2);
        int y_start = std::max(0, _odom_cells.second - _cells_y / 2);
        int x_stop = std::min(_cells_x, _odom_cells.first + _cells_x / 2);
        int y_stop = std::min(_cells_y, _odom_cells.second + _cells_y / 2);
        int delta_x = _cells_x / 2 - _odom_cells.first;
        int delta_y = _cells_y / 2 - _odom_cells.second;

        for (int y = y_start; y < y_stop; y++)
        {
            for (int x = x_start; x < x_stop; x++)
            {
                int x_new = x + delta_x;
                int y_new = y + delta_y;
                if (x_new >= 0 && x_new < _cells_x && y_new <= 0 && y_new < _cells_y)
                {
                    new_map->at<int8_t>(y_new, x_new) = _map->at<int8_t>(y, x);
                }
            }
        }
        _center.x = _odom.x;
        _center.y = _odom.y;
        _p_0.x = _center.x - _resolution * _cells_x * 0.5;
        _p_0.y = _center.y - _resolution * _cells_y * 0.5;
        _map = new_map;
    }
}

void Map::addLidar(const sensor_msgs::LaserScan &msg, int8_t coeff_block, int8_t coeff_unblock,
                   double max_dist)
{
    const std::lock_guard<std::mutex> lock(mu);
    double angle = static_cast<double>(msg.angle_min);
    int size = msg.ranges.size();
    for (int i = 0; i < size; i++)
    {
        double dist = 0.0;
        while (180.0 * abs(angle) / M_PI < 50)
        {
            double x = cos(angle) * dist;
            double y = sin(angle) * dist;
            if (dist < msg.ranges[i])
            {
                update_cell(x, y, coeff_unblock);
            }
            else
            {
                if (msg.ranges[i] <= max_dist)
                {
                    update_cell(x, y, coeff_block);
                }
                break;
            }
            dist += _resolution;
        }
        angle += msg.angle_increment;
    }
}

std::shared_ptr<nav_msgs::OccupancyGrid> Map::getMap(std::string frame_id)
{
    static uint32_t seq = 0;
    nav_msgs::OccupancyGrid grid;
    const std::lock_guard<std::mutex> lock(mu);

    grid.header.seq = seq;
    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = frame_id;

    grid.info.resolution = _resolution;
    grid.info.width = _cells_x;
    grid.info.height = _cells_y;
    grid.info.origin.position.x = _p_0.x;
    grid.info.origin.position.y = _p_0.y;

    grid.data.resize(_cells_x * _cells_y);
    for (int y = 0; y < _cells_y; y++)
    {
        for (int x = 0; x < _cells_x; x++)
        {
            grid.data[(_cells_y - 1 - y) * _cells_x + x] = _map->at<int8_t>(y, x);
        }
    }

    grid.info.map_load_time = ros::Time::now();

    return std::make_shared<nav_msgs::OccupancyGrid>(grid);
}
