#ifndef MAP_OVERLAY_HPP
#define MAP_OVERLAY_HPP

#include <l3xz_mapping/map/base/map_interface.hpp>

class MapOverlay : public MapInterface
{
  public:
    explicit MapOverlay(std::string name, int cells_x, int cells_y, double resolution,
                        double preview,
                        std::vector<std::shared_ptr<MapPostprocessing>> postprocessing,
                        std::string output_frame, std::vector<std::shared_ptr<MapInterface>> maps);
    virtual ~MapOverlay(){};
    MapOverlay(const MapOverlay &) = delete;
    MapOverlay(MapOverlay &&) = delete;
    MapOverlay &operator=(const MapOverlay &) = delete;
    MapOverlay &operator=(MapOverlay &&) = delete;

    void eval();
  private:
    std::vector<std::shared_ptr<MapInterface>> _maps;
    cv::Mat _concat_mat;
    
};
#endif // MAP_OVERLAY_HPP
