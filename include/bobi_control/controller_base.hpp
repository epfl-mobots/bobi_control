#ifndef BOBI_CONTROLLLER_BASE_HPP
#define BOBI_CONTROLLLER_BASE_HPP

#include <ros/ros.h>
#include <bobi_msgs/PoseStamped.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/MotorVelocities.h>

#include <cassert>

namespace bobi {

    class ControllerBase {
    public:
        ControllerBase(std::shared_ptr<ros::NodeHandle> nh, int id, std::string pose_topic)
            : _nh(nh),
              _ros_dt(0),
              _dt(0),
              _id(id),
              _pose_topic(pose_topic),
              _prev_stamp(ros::Time::now())
        {
            _pose_sub = _nh->subscribe(_pose_topic, 1, &ControllerBase::_pose_cb, this);
            _target_vel_sub = _nh->subscribe("target_velocities", 1, &ControllerBase::_target_velocities_cb, this);
            _target_pos_sub = _nh->subscribe("target_position", 1, &ControllerBase::_target_position_cb, this);

            _target_velocities.left = 0.;
            _target_velocities.right = 0.;
            _target_position.pose.xyz.x = -1;
            _target_position.pose.xyz.y = -1;
            _target_position.pose.rpy.yaw = 0.;
        }

        virtual void spin_once()
        {
            auto n = ros::Time::now();
            _ros_dt = n - _prev_stamp;
            _dt = std::max(_ros_dt.toSec(), 1e-6);
            _prev_stamp = n;
        }

        const std::string get_pose_topic() const
        {
            return _pose_topic;
        }

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

        void _pose_cb(const bobi_msgs::PoseVec::ConstPtr& msg)
        {
            std::lock_guard<std::mutex> guard(_pose_mtx);
            assert(_id >= 0);
            assert(_id <= msg->poses.size());
            if (msg->poses.size() == 0) {
                return;
            }

            _prev_pose = _pose;
            _pose = (*msg).poses[_id];
        }

        void _target_velocities_cb(const bobi_msgs::MotorVelocities::ConstPtr& msg)
        {
            std::lock_guard<std::mutex> guard(_tvel_mtx);
            _target_velocities = *msg;
        }

        void _target_position_cb(const bobi_msgs::PoseStamped::ConstPtr& msg)
        {
            std::lock_guard<std::mutex> guard(_tpos_mtx);
            _target_position = *msg;
        }

        std::shared_ptr<ros::NodeHandle> _nh;
        const int _id;
        const std::string _pose_topic;

        ros::Subscriber _target_vel_sub;
        std::mutex _tvel_mtx;
        bobi_msgs::MotorVelocities _target_velocities;

        ros::Subscriber _target_pos_sub;
        std::mutex _tpos_mtx;
        bobi_msgs::PoseStamped _target_position;

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