#ifndef BASE_MODEL_HPP
#define BASE_MODEL_HPP

#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

class ModelBase
{
    public:
        ModelBase(double resolution_m):_resolution_m(resolution_m), _pivot_x(0){};
        virtual ~ModelBase(){};
        ModelBase(const ModelBase&) = delete;
        ModelBase(ModelBase&&) = delete;
        ModelBase& operator=(const ModelBase&) = delete;
        ModelBase& operator=(ModelBase&&) = delete;

        virtual double probability(double x, double y, double y_point){}

        cv::Mat eval(double point_y_m);
        int getOffsetX() { return _pivot_x; }

    protected:
        cv::Mat _map;
        double _resolution_m;
        int _pivot_x;
};
#endif // BASE_MODEL_HPP
