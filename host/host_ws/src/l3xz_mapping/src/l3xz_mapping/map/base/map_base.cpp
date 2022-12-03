#include <l3xz_mapping/map/base/map_base.hpp>

template <class T>
MapBase<T>::MapBase(std::string name, int8_t coeff_block, int8_t coeff_unblock, int cells_x, int cells_y, double resolution, double preview, std::vector<std::shared_ptr<MapPostprocessing>> postprocessing, std::string tf_parent, std::string tf_child, int queue_size)
    : MapInterface(name, coeff_block, coeff_unblock, cells_x, cells_y, resolution, preview, postprocessing, tf_parent, tf_child),
      _queue_size(queue_size), _running(true)
{
    _thread = std::thread(&MapBase::worker, this);
}

template <class T>MapBase<T>::~MapBase()
{
    _running = false;
    _thread.join();
}

template <class T>
void MapBase<T>::add(const T &msg)
{
    std::lock_guard<std::mutex> lock(_mu);
    if(_messages.size() > _queue_size)
    {
        _messages.pop();
    }
    _messages.push(std::make_shared<T>(msg));
}

template <class T>
void MapBase<T>::worker()
{
    static Timer timer;
    while(_running)
    {
        timer.reset();
        _mu.lock();
        if(!_messages.empty())
        {
            eval(_messages.front());
            _messages.pop();
            if(_debug)
            {
                ROS_WARN("%s EVAL, %03fs, queue: %i", _name.c_str(), timer.getSeconds(), _messages.size());
            }
            timer.reset();
            postprocess();
            if(_debug)
            {
                ROS_WARN("%s POSTPROCESSING, %03fs", _name.c_str(), timer.getSeconds());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        update_map();
        _mu.unlock();
    }
}


