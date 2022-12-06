#ifndef MAP_BASE_HPP
#define MAP_BASE_HPP

#include <l3xz_mapping/map/base/map_interface.hpp>
#include <l3xz_mapping/timer.hpp>
#include <sensor_msgs/LaserScan.h>

template <class T> class MapBase : public MapInterface
{
  public:
    explicit MapBase(std::string name, int8_t coeff_block, int8_t coeff_unblock, int cells_x,
                     int cells_y, double resolution, double preview = 10.0,
                     std::vector<std::shared_ptr<MapPostprocessing>> postprocessing =
                 std::vector<std::shared_ptr<MapPostprocessing>>(),
             std::string tf_parent = "", std::string tf_child = "",
             std::string output_frame = "map", int queue_size = 10);
    virtual ~MapBase();
    MapBase(const MapBase &) = delete;
    MapBase(MapBase &&) = delete;
    MapBase &operator=(const MapBase &) = delete;
    MapBase &operator=(MapBase &&) = delete;

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
