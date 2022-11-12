#ifndef MODEL_LIDAR_HPP
#define MODEL_LIDAR_HPP

#include <l3xz_mapping/model/base/model_base.hpp>
#include <utility>

class ModelLidar : public ModelBase
{
    public:
        ModelLidar(double beamwidth_m, double maxrange_m, double affection_m, double p0, double p1, double p2, double resolution_m);
        ~ModelLidar();
        ModelLidar(const ModelLidar&) = delete;
        ModelLidar(ModelLidar&&) = delete;
        ModelLidar& operator=(const ModelLidar&) = delete;
        ModelLidar& operator=(ModelLidar&&) = delete;
        
        double probability(double x, double y, double y_point);

    private:
        double _beamwidth_m, _maxrange_m;
        double _affection_m;
        double _p0, _p1, _p2;
};
#endif
