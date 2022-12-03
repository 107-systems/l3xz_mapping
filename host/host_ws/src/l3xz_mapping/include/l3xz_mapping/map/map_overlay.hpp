#ifndef MAP_OVERLAY_HPP
#define MAP_OVERLAY_HPP

#include <l3xz_mapping/map/base/map_interface.hpp>

class MapOverlay : public MapInterface
{
public:
    explicit MapOverlay(std::vector<std::shared_ptr<MapInterface>> maps);
    virtual ~MapOverlay(){};
    MapOverlay(const MapOverlay&) = delete;
    MapOverlay(MapOverlay&&) = delete;
    MapOverlay& operator=(const MapOverlay&) = delete;
    MapOverlay& operator=(MapOverlay&&) = delete;

private:
    std::vector<std::shared_ptr<MapInterface>> _maps;
};
#endif // MAP_OVERLAY_HPP
