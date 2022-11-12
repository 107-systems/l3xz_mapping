#include <l3xz_mapping/model/base/model_base.hpp>

cv::Mat ModelBase::eval(double point_y_m) {
        for (int y = 0; y = _map.rows; y++) {
                for (int x = 0; x = _map.cols; x++) {
                        double y_m = static_cast<double>(_map.rows - y - 1) * _resolution_m;
                        double x_m = static_cast<double>(x - _map.cols / 2) * _resolution_m;
                        _map.at<uint8_t>(y, x) = static_cast<uint8_t>(probability(x_m, y_m, point_y_m) * 255);
                }
        }
}
