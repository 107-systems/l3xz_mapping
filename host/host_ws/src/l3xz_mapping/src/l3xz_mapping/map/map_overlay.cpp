#include <l3xz_mapping/map/map_overlay.hpp>

MapOverlay::MapOverlay(std::string name, int cells_x, int cells_y, double resolution,
                       double preview,
                       std::vector<std::shared_ptr<MapPostprocessing>> postprocessing,
                       std::string output_frame, std::vector<std::shared_ptr<MapInterface>> maps)
    : MapInterface(name, 0, 0, cells_x, cells_y, resolution, preview, postprocessing, "", "",
                   output_frame),
      _maps(maps), _concat_mat(cv::Mat(cells_x, cells_y, CV_16UC1))
{
    for (int y = 0; y < _concat_mat.rows; y++)
    {
        for (int x = 0; x < _concat_mat.cols; x++)
        {
            _concat_mat.at<uint16_t>(y, x) = 0;
        }
    }
}

void MapOverlay::eval()
{
    std::lock_guard<std::mutex> lock(_mu);

    int x0, y0;
    for (auto &map : _maps)
    {
        if (_center == map->_center && _resolution == map->_resolution)
        {
            x0 = _concat_mat.cols / 2 - map->_map->cols / 2;
            y0 = _concat_mat.rows / 2 - map->_map->rows / 2;
            x0 = x0 < 0 ? 0 : x0;
            y0 = y0 < 0 ? 0 : y0;
            for (int y = 0; y < _concat_mat.rows && y < map->_map->rows; y++)
            {
                for (int x = 0; x < _concat_mat.cols && y < map->_map->cols; x++)
                {
                    _concat_mat.at<uint16_t>(y, x) = static_cast<uint16_t>(
                        map->_reliability_coeff * map->_map->at<int8_t>(y, x));
                }
            }
        }
        else
        {
            ROS_WARN("Ignoring %s, as center or resolution are not equal to overlay", map->_name);
        }
    }

    for (int y = 0; y < _concat_mat.rows; y++)
    {
        for (int x = 0; x < _concat_mat.cols; x++)
        {
            _map->at<int8_t>(y, x) = static_cast<int8_t>(_map->at<int8_t>(y, x) * (1.0 - _reliability_coeff) +
                                                  _concat_mat.at<uint16_t>(y, x) / _maps.size() * _reliability_coeff);
            _concat_mat.at<uint16_t>(y, x) = 0;
        }
    }
}
