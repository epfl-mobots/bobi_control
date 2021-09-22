#ifndef BOBI_POSITION_CONTROL_HPP
#define BOBI_POSITION_CONTROL_HPP

#include <bobi_control/controller_base.hpp>

#include <dynamic_reconfigure/server.h>
#include <bobi_control/PositionControlConfig.h>

namespace bobi {
    class PositionControl : public ControllerBase {
    public:
        PositionControl(std::shared_ptr<ros::NodeHandle> nh, int id, std::string pose_topic)
            : ControllerBase(nh, id, pose_topic),
              _prev_error{0, 0},
              _integral{0, 0},
              _rotating(false)
        {
            dynamic_reconfigure::Server<bobi_control::PositionControlConfig>::CallbackType f;
            f = boost::bind(&PositionControl::_config_cb, this, _1, _2);
            _cfg_server.setCallback(f);

            _set_vel_pub = nh->advertise<bobi_msgs::MotorVelocities>("set_velocities", 1);
            // _set_vel_pub = nh->advertise<bobi_msgs::MotorVelocities>("target_velocities", 1);
        }

        void spin_once()
        {
            if (_prev_stamp > ros::Time(0)) {
                std::lock_guard<std::mutex> pose_guard(_pose_mtx);
                std::lock_guard<std::mutex> pos_guard(_tpos_mtx);

                auto euc_distance = [](const bobi_msgs::PoseStamped& lpose, const bobi_msgs::PoseStamped& rpose) {
                    return std::sqrt(std::pow(lpose.pose.xyz.x - rpose.pose.xyz.x, 2.) + std::pow(lpose.pose.xyz.y - rpose.pose.xyz.y, 2.));
                };

                if (euc_distance(_pose, _target_position) < _distance_threshold
                    || (_target_position.pose.xyz.x < 0 || _target_position.pose.xyz.y < 0)) {
                    _set_vel_pub.publish(bobi_msgs::MotorVelocities());
                    _integral = {0., 0.};
                    _prev_error = {0., 0.};
                    _rotating = false;
                    return;
                }

                if (_target_position.pose.xyz.x != _prev_target.pose.xyz.x
                    || _target_position.pose.xyz.y != _prev_target.pose.xyz.y) {
                    _integral = {0., 0.};
                    _prev_error = {0., 0.};
                    _rotating = false;
                }

                double yaw = _pose.pose.rpy.yaw;
                double vx = ((_pose.pose.xyz.x - _prev_pose.pose.xyz.x) / _dt);
                double vy = ((_pose.pose.xyz.y - _prev_pose.pose.xyz.y) / _dt);
                double v = std::sqrt(std::pow(vy, 2.) + std::pow(vx, 2.));
                double w = _angle_to_pipi(_pose.pose.rpy.yaw - _prev_pose.pose.rpy.yaw) / _dt;
                double l = 0.0451 * 2;
                double radius = 0.022;
                double theta = _angle_to_pipi(atan2(_target_position.pose.xyz.y - _pose.pose.xyz.y, _target_position.pose.xyz.x - _pose.pose.xyz.x));

                _current_velocities.left = (2 * v - w * l) / 2 * radius;
                _current_velocities.right = (2 * v + w * l) / 2 * radius;

                std::array<double, 2> error;
                error[0] = euc_distance(_pose, _target_position);
                error[1] = _angle_to_pipi(yaw - theta);

                // proportional
                std::array<double, 2> p;
                p[0] = _Kp[0] * error[0];
                p[1] = _Kp[1] * error[1];

                // integral
                _integral[0] += error[0] * _dt;
                _integral[1] += error[1] * _dt;
                std::array<double, 2> i;
                i[0] = _Ki[0] * _integral[0];
                i[1] = _Ki[1] * _integral[1];

                // derivative
                std::array<double, 2> d;
                d[0] = _Kd[0] * (error[0] - _prev_error[0]) / _dt;
                d[1] = _Kd[1] * (error[1] - _prev_error[1]) / _dt;

                // clipping values
                double v_hat = _clip(((p[0] + i[0] + d[0]) / _scaler[0]) / _dt, 0);
                double w_hat = _clip(((p[1] + i[1] + d[1]) / _scaler[1]) / _dt, 1);

                if (abs(error[1]) > _rotate_in_place_threshold) {
                    if (!_rotating) {
                        _rotating = true;
                        _integral[0] = 0;
                        _integral[1] = 0;
                    }
                    else {
                        v_hat = 0;
                    }
                }
                else {
                    if (_rotating) {
                        _rotating = false;
                        _integral[0] = 0;
                        _integral[1] = 0;
                    }
                }

                bobi_msgs::MotorVelocities new_velocities;
                new_velocities.left = (2 * v_hat - w_hat * l) / 2.;
                new_velocities.right = (2 * v_hat + w_hat * l) / 2.;

                if (_verbose) {
                    ROS_INFO("--- dt = %f", _dt);
                    ROS_INFO("Current velocities(left, right) = (%f, %f)", _current_velocities.left, _current_velocities.right);
                    ROS_INFO("Current velocities(v, w) = (%f, %f)", v, w);
                    ROS_INFO("Target position(x, y) = (%f, %f)", _target_position.pose.xyz.x, _target_position.pose.xyz.y);
                    ROS_INFO("Current position(x, y) = (%f, %f)", _pose.pose.xyz.x, _pose.pose.xyz.y);
                    ROS_INFO("New velocities(left, right) = (%f, %f)", new_velocities.left, new_velocities.right);
                    ROS_INFO("New velocities(v, w) = (%f, %f)", v_hat, w_hat);
                    ROS_INFO("Error(linear, angular) = (%f, %f)", error[0], error[1]);
                }

                _set_vel_pub.publish(new_velocities);
                _prev_error = error;
                _prev_target = _targe_position;
            }
        }

    protected:
        void _config_cb(bobi_control::PositionControlConfig& config, uint32_t level)
        {
            ROS_INFO("Updated %s config", __func__);
            _Kp = {config.Kp_v, config.Kp_w};
            _Ki = {config.Ki_v, config.Ki_w};
            _Kd = {config.Kd_v, config.Kd_w};
            _lb = {config.lb_v, config.lb_w};
            _ub = {config.ub_v, config.ub_w};
            _scaler = {config.scaler_v, config.scaler_w};
            _distance_threshold = config.distance_threshold;
            _rotate_in_place_threshold = config.rotate_in_place_threshold;
            _verbose = config.verbose;
        }

        double _clip(double val, size_t idx)
        {
            val = std::min(val, _ub[idx]);
            val = std::max(val, _lb[idx]);
            return val;
        }

        dynamic_reconfigure::Server<bobi_control::PositionControlConfig> _cfg_server;
        ros::Publisher _set_vel_pub;

        std::array<double, 2> _Kp, _Kd, _Ki;
        std::array<double, 2> _prev_error;
        std::array<double, 2> _integral;
        std::array<double, 2> _lb, _ub;
        std::array<double, 2> _scaler;
        double _distance_threshold;
        double _rotate_in_place_threshold;
        bool _verbose;
        bool _rotating;

        bobi_msgs::PoseStamped _current_position;
        bobi_msgs::MotorVelocities _current_velocities;
        bobi_msgs::PoseStamped _prev_target;
    };

} // namespace bobi

#endif
