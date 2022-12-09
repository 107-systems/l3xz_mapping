#ifndef MAP_POSTPROCESSING_HPP
#define MAP_POSTPROCESSING_HPP

#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

class MapPostprocessing
{
  public:
    explicit MapPostprocessing(){};
    virtual ~MapPostprocessing() = default;
    MapPostprocessing(const MapPostprocessing &) = delete;
    MapPostprocessing(MapPostprocessing &&) = delete;
    MapPostprocessing &operator=(const MapPostprocessing &) = delete;
    MapPostprocessing &operator=(MapPostprocessing &&) = delete;

    virtual std::shared_ptr<cv::Mat> eval(std::shared_ptr<cv::Mat> input) { return input; }
};
#endif
