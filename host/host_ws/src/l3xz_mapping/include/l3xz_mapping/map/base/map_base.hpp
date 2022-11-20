#ifndef MAP_BASE_HPP
#define MAP_BASE_HPP

#include <sensor_msgs/LaserScan.h>
#include <l3xz_mapping/map/base/map_interface.hpp>

template<class T>
class MapBase : public MapInterface
{
       public:
        explicit MapBase(int8_t coeff_block, int8_t coeff_unblock, int cells_x, int cells_y, double resolution, double preview = 10.0, int queue_size = 10);
        virtual ~MapBase();
        MapBase(const MapBase&) = delete;
        MapBase(MapBase&&) = delete;
        MapBase& operator=(const MapBase&) = delete;
        MapBase& operator=(MapBase&&) = delete;

        void add(const T &msg);
        virtual void eval(std::shared_ptr<T> msg){};

       protected:
        std::thread _thread;
        bool _running;
        void worker();
        std::queue<std::shared_ptr<T>> _messages;
        int _queue_size;
};

template class MapBase<sensor_msgs::LaserScan>;
#endif
