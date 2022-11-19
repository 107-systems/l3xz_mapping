#ifndef MAP_LIDAR_HPP
#define MAP_LIDAR_HPP

#include <l3xz_mapping/map/base/map_base.hpp>

class MapLidar : public MapBase {
       public:
        MapLidar(int8_t coeff_block, int8_t coeff_unblock, double max_dist,
                 int cells_x, int cells_y, double resolution,
                 double preview = 10.0)
            : MapBase(coeff_block, coeff_unblock, cells_x, cells_y, resolution,
                  preview),
              _max_dist(max_dist){};

        void addLidar(const sensor_msgs::LaserScan &msg) {
                const std::lock_guard<std::mutex> lock(mu);
                double angle = static_cast<double>(msg.angle_min);
                int size = msg.ranges.size();
                for (int i = 0; i < size; i++) {
                        double dist = 0.0;
                        while (180.0 * abs(angle) / M_PI < 50) {
                                double x = cos(angle) * dist;
                                double y = sin(angle) * dist;
                                if (dist < msg.ranges[i]) {
                                        update_cell(x, y, _coeff_unblock);
                                } else {
                                        if (msg.ranges[i] <= _max_dist) {
                                                update_cell(x, y, _coeff_block);
                                        }
                                        break;
                                }
                                dist += _resolution;
                        }
                        angle += msg.angle_increment;
                }
        }

       private:
        double _max_dist;
};
#endif
