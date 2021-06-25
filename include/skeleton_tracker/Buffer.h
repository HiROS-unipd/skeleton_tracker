#ifndef hiros_skeleton_tracker_Buffer_h
#define hiros_skeleton_tracker_Buffer_h

// Standard dependencies
#include <map>

// ROS dependencies
#include "ros/time.h"

namespace hiros {
  namespace track {

    template <class T>
    class Buffer
    {
    public:
      Buffer(const double& t_dt_epsilon = 0.001);

      size_t size() const;
      bool empty() const;

      void push_back(const T& t_msg);
      void pop_back();
      void erase_until_time(const ros::Time& t_time);

      ros::Time get_src_time() const;
      std::string get_src_frame() const;
      T get_skeleton_group() const;

    private:
      const double DT_EPSILON{0.001};
      std::map<ros::Time, T> m_buffer{};
    };

    template <class T>
    Buffer<T>::Buffer(const double& t_dt_epsilon)
      : DT_EPSILON(t_dt_epsilon)
    {}

    template <class T>
    size_t Buffer<T>::size() const
    {
      return m_buffer.size();
    }

    template <class T>
    bool Buffer<T>::empty() const
    {
      return m_buffer.size() == 0;
    }

    template <class T>
    void Buffer<T>::push_back(const T& t_msg)
    {
      m_buffer.emplace(t_msg->src_time, t_msg);
    }

    template <class T>
    void Buffer<T>::pop_back()
    {
      m_buffer.erase(m_buffer.begin());
    }

    template <class T>
    void Buffer<T>::erase_until_time(const ros::Time& t_time)
    {
      std::erase_if(m_buffer, [&](const auto& e) { return ((e.first - t_time).toSec() < DT_EPSILON); });
    }

    template <class T>
    ros::Time Buffer<T>::get_src_time() const
    {
      return m_buffer.begin()->second->src_time;
    }

    template <class T>
    std::string Buffer<T>::get_src_frame() const
    {
      return m_buffer.begin()->second->src_frame;
    }

    template <class T>
    T Buffer<T>::get_skeleton_group() const
    {
      return m_buffer.begin()->second;
    }

  } // namespace track
} // namespace hiros

#endif
