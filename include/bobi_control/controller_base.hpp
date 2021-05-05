#ifndef BOBI_CONTROLLLER_BASE_HPP
#define BOBI_CONTROLLLER_BASE_HPP

#include <ros/ros.h>
#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/MotorVelocities.h>

namespace bobi {

    class ControllerBase {
    public:
        ControllerBase(std::shared_ptr<ros::NodeHandle> nh)
            : _nh(nh),
              _ros_dt(0),
              _dt(0)
        {
            _pose_sub = _nh->subscribe("robot_pose", 1, &ControllerBase::_pose_cb, this);
            _target_vel_sub = _nh->subscribe("target_velocities", 1, &ControllerBase::_target_velocities_cb, this);
        }

        virtual void spin_once() = 0;

    protected:
        template <typename T>
        int _sgn(T val)
        {
            return (T(0) < val) - (val < T(0));
        }

        double _angle_to_pipi(double angle)
        {
            while (true) {
                if (angle < -M_PI) {
                    angle += 2. * M_PI;
                }
                if (angle > M_PI) {
                    angle -= 2. * M_PI;
                }
                if (abs(angle) <= M_PI) {
                    break;
                }
            }
            return angle;
        }

        void _pose_cb(const bobi_msgs::PoseStamped::ConstPtr& msg)
        {
            std::lock_guard<std::mutex> guard(_pose_mtx);
            _ros_dt = msg->header.stamp - _prev_stamp;
            _dt = std::max(_ros_dt.toSec(), 1e-6);
            _prev_stamp = msg->header.stamp;
            _prev_pose = _pose;
            _pose = *msg;
        }

        void _target_velocities_cb(const bobi_msgs::MotorVelocities::ConstPtr& msg)
        {
            std::lock_guard<std::mutex> guard(_tvel_mtx);
            _target_velocities = *msg;
        }

        std::shared_ptr<ros::NodeHandle> _nh;

        ros::Subscriber _target_vel_sub;
        std::mutex _tvel_mtx;
        bobi_msgs::MotorVelocities _target_velocities;

        ros::Subscriber _pose_sub;
        std::mutex _pose_mtx;
        bobi_msgs::PoseStamped _pose;
        bobi_msgs::PoseStamped _prev_pose;

        ros::Time _prev_stamp;
        ros::Duration _ros_dt;
        double _dt;
    };

} // namespace bobi

#endif