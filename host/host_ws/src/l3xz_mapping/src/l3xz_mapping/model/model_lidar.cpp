#include <l3xz_mapping/model/model_lidar.hpp>

ModelLidar::ModelLidar(double beamwidth_m, double maxrange_m,
                       double affection_m, double p0, double p1, double p2,
                       double resolution_m)
    : ModelBase(resolution_m),
      _beamwidth_m(beamwidth_m),
      _maxrange_m(maxrange_m),
      _affection_m(affection_m),
      _p0(p0),
      _p1(p1),
      _p2(p2) {
        int rows = static_cast<int>(maxrange_m / resolution_m);
        int cols = static_cast<int>(beamwidth_m / resolution_m);
        _map = cv::Mat(rows > 0 ? rows : 1, cols > 0 ? cols : 1, CV_8UC1);
}

ModelLidar::~ModelLidar() {}

double ModelLidar::probability(double x, double y, double y_point) {
    if(y < y_point - _affection_m)
    {
        return _p0;
    }
    else if(y < y_point)
    {
        return ((_p1 - _p0) / _affection_m) * y + _p0; 
    }
    else if(y < y_point + _affection_m)
    {
        return ((_p2 - _p1) / _affection_m) * y + _p1;
    }
    else
    {
        return _p2;
    }
    return 0.0;
}
