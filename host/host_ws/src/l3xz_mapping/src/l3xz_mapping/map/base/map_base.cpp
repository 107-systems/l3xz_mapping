#include <l3xz_mapping/map/base/map_base.hpp>

template <class T>
MapBase<T>::MapBase(int8_t coeff_block, int8_t coeff_unblock, int cells_x, int cells_y, double resolution, double preview, int queue_size)
    : MapInterface(coeff_block, coeff_unblock, cells_x, cells_y, resolution, preview),
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
    while(_running)
    {
        _mu.lock();
        if(!_messages.empty())
        {
            eval(_messages.front());
            _messages.pop();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        _mu.unlock();
    }
}


