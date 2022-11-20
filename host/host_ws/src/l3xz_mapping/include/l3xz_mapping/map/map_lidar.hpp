#ifndef MAP_LIDAR_HPP
#define MAP_LIDAR_HPP

#include <l3xz_mapping/map/base/map_base.hpp>

class MapLidar : public MapBase<sensor_msgs::LaserScan> {
       public:
        explicit MapLidar(int8_t coeff_block, int8_t coeff_unblock, double max_dist, double clamp_deg,
                 int cells_x, int cells_y, double resolution,
                 double preview = 10.0)
            : MapBase<sensor_msgs::LaserScan>(coeff_block, coeff_unblock, cells_x, cells_y, resolution,
                  preview),
              _max_dist(max_dist), _clamp_deg(clamp_deg){};

        virtual ~MapLidar() {};
        MapLidar(const MapLidar&) = delete;
        MapLidar(MapLidar&&) = delete;
        MapLidar& operator=(const MapLidar&) = delete;
        MapLidar& operator=(MapLidar&&) = delete;

        void eval(std::shared_ptr<sensor_msgs::LaserScan> msg) {
                double angle = static_cast<double>(msg->angle_min);
                int size = msg->ranges.size();
                for (int i = 0; i < size; i++) {
                        double dist = 0.0;
                        while (180.0 * abs(angle) / M_PI < _clamp_deg) {
                                double x = cos(angle) * dist;
                                double y = sin(angle) * dist;
                                if (dist < msg->ranges[i]) {
                                        update_cell(x, y, _coeff_unblock);
                                } else {
                                        if (msg->ranges[i] <= _max_dist) {
                                                update_cell(x, y, _coeff_block);
                                        }
                                        break;
                                }
                                dist += _resolution;
                        }
                        angle += msg->angle_increment;
                }
        }

       private:
        double _max_dist;
        double _clamp_deg;
};
#endif
