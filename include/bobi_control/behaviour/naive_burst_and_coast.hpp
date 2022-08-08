#ifndef NAIVE_BURST_AND_COAST_HPP
#define NAIVE_BURST_AND_COAST_HPP

#include <bobi_control/controller_base.hpp>

#include <bobi_control/NaiveBurstAndCoastConfig.h>

#include <bobi_msgs/MaxAcceleration.h>
#include <bobi_msgs/DutyCycle.h>
#include <bobi_msgs/Pose.h>
#include <bobi_msgs/PoseVec.h>
#include <bobi_msgs/SpeedEstimateVec.h>

#include <tools/random/random_generator.hpp>

#include <dynamic_reconfigure/server.h>

#include <Eigen/Core>

namespace bobi {
    namespace behaviour {

        namespace defaults {
            struct RummyIndividualParams {
                float radius = 20.;

                // interactions
                int perceived_agents = 1;
                float gamma_rand = 0.3;
                float gamma_wall = 0.23;
                float gamma_sym = 0.;
                float gamma_asym = 0.05;
                float wall_interaction_range = 6.;

                float dw = 6.;
                float dc = 1.;
                float alpha_w = 0.;
                float gamma_attraction = 0.3;
                float gamma_alignment = 0.3;

                bool iuturn = true;
                float duturn = 6.;
                float pjump = 0.;
                float psi_c = 0.25;

                // kicks
                float vmean = 43.;
                float vmin = 1.;
                float vmem = 0.9;
                float vmem12 = 0.5;
                float vcut = 35.;
                float taumean = 0.52;
                float taumin = 0.2;
                float tau0 = 0.8;

                // simu
                int itermax = 10;
            };
        } // namespace defaults

        enum Order {
            DECREASING,
            INCREASING
        };

        float ran3()
        {
            return tools::random_in_range(0., 1.);
        }

        using state_t = std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>;
        using const_state_t = const std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>&;

        class NaiveBurstAndCoast : public ControllerBase {
        public:
            NaiveBurstAndCoast(std::shared_ptr<ros::NodeHandle> nh, int id, std::string pose_topic, const float wheel_radius, const float wheel_distance, defaults::RummyIndividualParams params = defaults::RummyIndividualParams())
                : ControllerBase(nh, id, pose_topic),
                  _wheel_radius(wheel_radius),
                  _wheel_distance(wheel_distance),
                  _prev_error{0, 0},
                  _integral{0, 0},
                  _rotating(false),
                  _using_robot_motor_feedback(false),
                  _params(params),
                  _total_time(0.),
                  _setup_center_top{0., 0.},
                  _setup_center_bottom{0., 0.},
                  _iters(0),
                  _lure_rescue(false),
                  _mean_speed(0.)
            {
                // Initialize ROS interface
                dynamic_reconfigure::Server<bobi_control::NaiveBurstAndCoastConfig>::CallbackType f;
                f = boost::bind(&NaiveBurstAndCoast::_config_cb, this, _1, _2);
                _cfg_server.setCallback(f);

                _set_vel_pub = nh->advertise<bobi_msgs::MotorVelocities>("set_velocities", 1);
                _set_target_pos_pub = nh->advertise<bobi_msgs::PoseStamped>("target_position", 1);
                // _set_vel_pub = nh->advertise<bobi_msgs::MotorVelocities>("target_velocities", 1);
                _cur_vel_sub = nh->subscribe("current_velocities", 1, &NaiveBurstAndCoast::_current_vel_cb, this);
                _cur_vel_sub = nh->subscribe("filtered_poses", 1, &NaiveBurstAndCoast::_individual_poses_cb, this);
                _speed_estimates_sub = nh->subscribe("speed_estimates", 1, &NaiveBurstAndCoast::_speed_estimates_cb, this);

                _max_accel_cli = nh->serviceClient<bobi_msgs::MaxAcceleration>("set_max_acceleration");
                _set_duty_cycle_cli = nh->serviceClient<bobi_msgs::DutyCycle>("set_duty_cycle");

                assert(nh->getParam("top_camera/pix2m", _top_pix2m));
                assert(nh->getParam("bottom_camera/pix2m", _bottom_pix2m));
                double cx, cy;
                assert(nh->getParam("top_camera/camera_px_width_undistorted", cx));
                assert(nh->getParam("top_camera/camera_px_height_undistorted", cy));
                _setup_center_top = {(cx / 2) * _top_pix2m, (cy / 2) * _top_pix2m};
                bobi_msgs::PoseStamped center_top;
                center_top.pose.xyz.x = _setup_center_top[0];
                center_top.pose.xyz.y = _setup_center_top[1];
                bobi_msgs::PoseStamped center_bottom = convert_top2bottom(center_top);
                _setup_center_bottom = {center_bottom.pose.xyz.x, center_bottom.pose.xyz.y};

                // Initialize model params and the first kick
                _t0 = 0;
                _tau = 0;
            }

            void spin_once()
            {
                ControllerBase::spin_once();

                auto euc_distance = [](const bobi_msgs::PoseStamped& lpose, const bobi_msgs::PoseStamped& rpose) {
                    return std::sqrt(std::pow(lpose.pose.xyz.x - rpose.pose.xyz.x, 2.) + std::pow(lpose.pose.xyz.y - rpose.pose.xyz.y, 2.));
                };

                if (_prev_stamp > ros::Time(0)) {
                    std::lock_guard<std::mutex> pose_guard(_pose_mtx);
                    std::lock_guard<std::mutex> pos_guard(_tpos_mtx);

                    _total_time += _dt;

                    if (euc_distance(_pose, _target_position) < _distance_threshold
                        || (_target_position.pose.xyz.x < 0 || _target_position.pose.xyz.y < 0) && !_lure_rescue) {
                        // _set_vel_pub.publish(bobi_msgs::MotorVelocities());
                        _integral = {0., 0.};
                        _prev_error = {0., 0.};
                        _rotating = false;
                        _t0 = _total_time;
                        _tau = 0;
                    }

                    if (_target_position.pose.xyz.x != _prev_target.pose.xyz.x
                        || _target_position.pose.xyz.y != _prev_target.pose.xyz.y) {
                        _integral = {0., 0.};
                        _prev_error = {0., 0.};
                        _rotating = false;
                    }

                    if (_pose.pose.xyz.x == _prev_pose.pose.xyz.x
                        && _pose.pose.xyz.y == _prev_pose.pose.xyz.y
                        && _prev_v[0] != 0 && _prev_v[1] != 0) {
                        _pose.pose.xyz.x += _prev_v[0] * _dt;
                        _pose.pose.xyz.y += _prev_v[1] * _dt;
                    }

                    if (!_individual_poses.size()) {
                        return;
                    }

                    _pose_in_cm = _individual_poses[_id];
                    bobi_msgs::PoseStamped rpose_in_cm = _pose;
                    rpose_in_cm.pose.xyz.x -= _setup_center_bottom[0];
                    rpose_in_cm.pose.xyz.y -= _setup_center_bottom[1];
                    rpose_in_cm.pose.xyz.x *= 100;
                    rpose_in_cm.pose.xyz.y *= 100;

                    double lure_vs_robot_dist = euc_distance(_pose_in_cm, rpose_in_cm);
                    if (lure_vs_robot_dist > 2.5) {
                        ROS_INFO("Rescuing the lure");
                        _lure_rescue = true;
                    }

                    if (lure_vs_robot_dist < 0.5) {
                        if (_lure_rescue) {
                            ROS_INFO("Lure rescued!");
                            _t0 = _total_time;
                            _tau = 0;
                        }
                        _lure_rescue = false;
                    }

                    double rotate_in_place_threshold_ub;
                    if (!_lure_rescue) {
                        float r = std::sqrt(_pose_in_cm.pose.xyz.x * _pose_in_cm.pose.xyz.x + _pose_in_cm.pose.xyz.y * _pose_in_cm.pose.xyz.y);

                        if (std::abs(r - _params.radius) <= 2.) {
                            rotate_in_place_threshold_ub = 0.7;
                        }
                        else {
                            rotate_in_place_threshold_ub = _rotate_in_place_threshold_ub;
                        }

                        if (_total_time >= _t0 + _tau) {

                            if (!_kick()) {
                                if ((_speeds[_id] * 100) * _dt + r > _params.radius) {
                                    // _new_velocities.left = 0.1;
                                    // _new_velocities.right = -0.1;
                                    // _set_vel_pub.publish(_new_velocities);
                                }
                                ++_iters;
                                return;
                            }
                            else {
                                _t0 = _total_time;
                                _iters = 0;
                            }

                            bobi_msgs::PoseStamped candidate_pose;

                            float expt = std::exp(-_tau / _params.tau0);
                            float dl = _speed * _params.tau0 * (1. - expt) + _params.dc;
                            candidate_pose.pose.xyz.x = _pose_in_cm.pose.xyz.x + dl * cos(_traj_pose.rpy.yaw);
                            candidate_pose.pose.xyz.y = _pose_in_cm.pose.xyz.y + dl * sin(_traj_pose.rpy.yaw);
                            _mean_speed = dl / _tau;
                            float r_new = std::sqrt(candidate_pose.pose.xyz.x * candidate_pose.pose.xyz.x + candidate_pose.pose.xyz.y * candidate_pose.pose.xyz.y);

                            if (r_new > _params.radius) {
                                _target_position.pose.xyz.x = _pose_in_cm.pose.xyz.x + (_params.radius - r) * cos(_traj_pose.rpy.yaw);
                                _target_position.pose.xyz.y = _pose_in_cm.pose.xyz.y + (_params.radius - r) * sin(_traj_pose.rpy.yaw);
                                _mean_speed = std::abs(_params.radius - r) / _tau;
                                ROS_ERROR(">>> Target position out of bounds");
                            }
                            else {
                                _target_position = candidate_pose;
                            }
                            _target_position.pose.rpy.yaw = _traj_pose.rpy.yaw;

                            // Take care of scale
                            _mean_speed /= 100.; // speed in m/s
                            _target_position.pose.xyz.x /= 100.; // position in m
                            _target_position.pose.xyz.y /= 100.; // position in m
                            _target_position.pose.xyz.x += _setup_center_bottom[0]; // offset by center x coordinate
                            _target_position.pose.xyz.y += _setup_center_bottom[1]; // offset by center y coordinate

                            _target_position.header.stamp = ros::Time::now();
                            _set_target_pos_pub.publish(_target_position); // ! this is slightly weird to do and it's mostly for the viz
                        }
                    }
                    else {
                        _target_position = _pose_in_cm;
                        _mean_speed = 4;
                        _t0 = _total_time;
                        _tau = 0;

                        // Take care of scale
                        _mean_speed /= 100.; // speed in m/s
                        _target_position.pose.xyz.x /= 100.; // position in m
                        _target_position.pose.xyz.y /= 100.; // position in m
                        _target_position.pose.xyz.x += _setup_center_bottom[0]; // offset by center x coordinate
                        _target_position.pose.xyz.y += _setup_center_bottom[1]; // offset by center y coordinate

                        _target_position.header.stamp = ros::Time::now();
                        _set_target_pos_pub.publish(_target_position); // ! this is slightly weird to do and it's mostly for the viz
                    }

                    ROS_INFO("Mean speed: %f", _mean_speed);
                    // _ub[0] = _mean_speed;

                    // // Take care of scale
                    // bool no_publish = false;

                    // _mean_speed /= 100.; // speed in m/s
                    // _target_position.pose.xyz.x /= 100.; // position in m
                    // _target_position.pose.xyz.y /= 100.; // position in m
                    // _target_position.pose.xyz.x += _setup_center_bottom[0]; // offset by center x coordinate
                    // _target_position.pose.xyz.y += _setup_center_bottom[1]; // offset by center y coordinate

                    // _target_position.header.stamp = ros::Time::now();
                    // if (!no_publish) {
                    //     _set_target_pos_pub.publish(_target_position); // ! this is slightly weird to do and it's mostly for the viz
                    // }
#if 1

                    double yaw = _pose.pose.rpy.yaw;
                    double vx = ((_pose.pose.xyz.x - _prev_pose.pose.xyz.x) / _dt);
                    double vy = ((_pose.pose.xyz.y - _prev_pose.pose.xyz.y) / _dt);
                    double v = std::sqrt(std::pow(vy, 2.) + std::pow(vx, 2.));
                    _prev_v = {vx, vy, v};
                    double motor_v = std::sqrt(std::pow(_current_velocities.left, 2.) + std::pow(_current_velocities.right, 2.));
                    double w = _angle_to_pipi(_pose.pose.rpy.yaw - _prev_pose.pose.rpy.yaw) / _dt;
                    const float l = _wheel_distance;
                    const float radius = _wheel_radius;
                    double theta = _angle_to_pipi(atan2(_target_position.pose.xyz.y - _pose.pose.xyz.y, _target_position.pose.xyz.x - _pose.pose.xyz.x));

                    std::array<double, 2> error;
                    // error[0] = euc_distance(_pose, _target_position);
                    error[0] = std::abs(motor_v - _mean_speed);
                    error[1] = _angle_to_pipi(yaw - theta);

                    if (abs(error[1]) > rotate_in_place_threshold_ub) {
                        if (!_rotating) {
                            _rotating = true;
                            _integral[0] = 0;
                            _integral[1] = 0;
                        }
                    }
                    else {
                        if (_rotating && abs(error[1]) <= _rotate_in_place_threshold_lb) {
                            _rotating = false;
                            _integral[0] = 0;
                            _integral[1] = 0;
                        }
                    }

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
                    // double v_hat = _clip(((p[0] + i[0] + d[0]) / _scaler[0]) / _dt, 0);
                    double v_hat = _clip(((p[0] + i[0] + d[0]) / _scaler[0]), 0);
                    // double v_hat = _clip(_mean_speed, 0);
                    double w_hat = _clip(((p[1] + i[1] + d[1]) / _scaler[1]) / _dt, 1);

                    if (_rotating) {
                        v_hat = 0.;
                    }

                    _new_velocities.left = (2 * v_hat - w_hat * l) / 2.;
                    _new_velocities.right = (2 * v_hat + w_hat * l) / 2.;

                    if (_verbose) {
                        if (!_using_robot_motor_feedback) {
                            _current_velocities.left = (2 * v - w * l) / 2 * radius;
                            _current_velocities.right = (2 * v + w * l) / 2 * radius;
                        }
                        else {
                            _using_robot_motor_feedback = false;
                        }
                        _prev_v = {vx, vy, v};

                        ROS_INFO("--- dt = %f", _dt);
                        ROS_INFO("Current velocities(left, right) = (%f, %f)", _current_velocities.left, _current_velocities.right);
                        ROS_INFO("Current velocities(v, w) = (%f, %f)", v, w);
                        ROS_INFO("Target position(x, y) = (%f, %f)", _target_position.pose.xyz.x, _target_position.pose.xyz.y);
                        ROS_INFO("Current position(x, y) = (%f, %f)", _pose.pose.xyz.x, _pose.pose.xyz.y);
                        ROS_INFO("New velocities(left, right) = (%f, %f)", _new_velocities.left, _new_velocities.right);
                        ROS_INFO("New velocities(v, w) = (%f, %f)", v_hat, w_hat);
                        ROS_INFO("Error(linear, angular) = (%f, %f)", error[0], error[1]);
                    }

                    _set_vel_pub.publish(_new_velocities);
                    _prev_error = error;
                    _prev_target = _target_position;
#endif
                }
            }

        protected:
            void _current_vel_cb(const bobi_msgs::MotorVelocities::ConstPtr& motor_velocities)
            {
                _using_robot_motor_feedback = true;
                _current_velocities.left = motor_velocities->left / 100.;
                _current_velocities.right = motor_velocities->right / 100.;
            }

            void _individual_poses_cb(const bobi_msgs::PoseVec::ConstPtr& pose_vec)
            {
                _individual_poses.clear();
                for (size_t i = 0; i < pose_vec->poses.size(); ++i) {
                    bobi_msgs::PoseStamped pose = pose_vec->poses[i];
                    pose = convert_top2bottom(pose);
                    pose.pose.xyz.x -= _setup_center_bottom[0];
                    pose.pose.xyz.y -= _setup_center_bottom[1];
                    pose.pose.xyz.x *= 100;
                    pose.pose.xyz.y *= 100;
                    _individual_poses.push_back(pose);
                }
            }

            void _speed_estimates_cb(const bobi_msgs::SpeedEstimateVec::ConstPtr& speed_vec)
            {
                _speeds.clear();
                if (speed_vec->speeds.size() > 0) {
                    std::copy(speed_vec->speeds.begin(), speed_vec->speeds.end(),
                        std::back_inserter(_speeds));
                }
            }

            void _config_cb(bobi_control::NaiveBurstAndCoastConfig& config, uint32_t level)
            {
                ROS_INFO("Updated %s config", __func__);
                _Kp = {config.Kp_v, config.Kp_w};
                _Ki = {config.Ki_v, config.Ki_w};
                _Kd = {config.Kd_v, config.Kd_w};
                _lb = {config.lb_v, config.lb_w};
                _ub = {config.ub_v, config.ub_w};
                _scaler = {config.scaler_v, config.scaler_w};
                _distance_threshold = config.distance_threshold;
                _rotate_in_place_threshold_ub = config.rotate_in_place_threshold_ub;
                _rotate_in_place_threshold_lb = config.rotate_in_place_threshold_lb;
                _verbose = config.verbose;

                _params.radius = config.radius * 100;
                _params.iuturn = config.iuturn;
                _params.perceived_agents = config.perceived_agents;
                _params.gamma_rand = config.gamma_rand;
                _params.gamma_wall = config.gamma_wall;
                _params.gamma_sym = config.gamma_sym;
                _params.gamma_asym = config.gamma_asym;
                _params.wall_interaction_range = config.wall_interaction_range;
                _params.dw = config.dw;
                _params.dc = config.dc;
                _params.gamma_attraction = config.gamma_attraction;
                _params.gamma_alignment = config.gamma_alignment;
                _params.duturn = config.duturn;
                _params.pjump = config.pjump;
                _params.psi_c = config.psi_c;
                _params.vmean = config.vmean;
                _params.vmin = config.vmin;
                _params.vmem = config.vmem;
                _params.vcut = config.vcut;
                _params.taumean = config.taumean;
                _params.taumin = config.taumin;
                _params.tau0 = config.tau0;
                _params.itermax = config.itermax;
            }

            double _clip(double val, size_t idx)
            {
                val = std::min(val, _ub[idx]);
                val = std::max(val, _lb[idx]);
                return val;
            }

            dynamic_reconfigure::Server<bobi_control::NaiveBurstAndCoastConfig> _cfg_server;
            ros::Publisher _set_vel_pub;
            ros::Publisher _set_target_pos_pub;
            ros::Subscriber _cur_vel_sub;
            ros::Subscriber _individual_poses_sub;
            ros::Subscriber _speed_estimates_sub;

            ros::ServiceClient _max_accel_cli;
            ros::ServiceClient _set_duty_cycle_cli;

            std::array<double, 2> _Kp, _Kd, _Ki;
            std::array<double, 2> _prev_error;
            std::array<double, 2> _integral;
            std::array<double, 2> _lb, _ub;
            std::array<double, 2> _scaler;
            std::array<double, 2> _setup_center_top;
            std::array<double, 2> _setup_center_bottom;
            double _distance_threshold;
            double _rotate_in_place_threshold_ub;
            double _rotate_in_place_threshold_lb;
            bool _verbose;
            bool _lure_rescue;
            float _mean_speed;

            const float _wheel_radius;
            const float _wheel_distance;
            bool _rotating;
            bool _using_robot_motor_feedback;

            bobi_msgs::PoseStamped _pose_in_cm;
            bobi_msgs::MotorVelocities _new_velocities;
            bobi_msgs::MotorVelocities _current_velocities;
            bobi_msgs::PoseStamped _prev_target;
            std::array<double, 3> _prev_v;

            std::vector<bobi_msgs::PoseStamped> _individual_poses;
            std::vector<double> _speeds;

            double _top_pix2m;
            double _bottom_pix2m;

        private:
            bool _kick()
            {
                ++_num_kicks;

                double speed_in_cm = _speeds[_id] * 100;
                if (_iters > _params.itermax) {
                    speed_in_cm = 10.;
                }
                float v0old = speed_in_cm;
                float vold = speed_in_cm * std::exp(-_tau / _params.tau0);

                auto state = _compute_state();
                auto neighs = _sort_neighbours(std::get<0>(state), _id, Order::INCREASING); // by distance

                float dphi_int, fw;
                std::tie(dphi_int, fw) = _compute_interactions(state, neighs);

                /**
                 *
                 * Step the model, this is the actual "movement" of the fish
                 *
                 **/

                int iter = 0;
                float phi_new, r_new;
                int cn_idx = _id; // ! perhaps instead of using the focal id I should simple bypass usages of cn_idx
                if (neighs.size()) {
                    cn_idx = neighs[0]; // closest_neigh idx
                }

                auto start_stamp = ros::Time::now();
                auto stop_stamp = ros::Time::now();
                // do {
                //     _t0 += stop_stamp.toSec() - start_stamp.toSec();
                // do {
                float prob = ran3();
                if (prob < _params.vmem) {
                    if (prob < _params.vmem * _params.vmem12) {
                        _speed = _speeds[cn_idx] * 100;
                    }
                    else {
                        _speed = v0old;
                    }
                }
                else {
                    // if (vold > _params.vmean) {
                    //     _speed = vold;
                    // }
                    // else {
                    //     _speed = vold - (_params.vmean - vold) * log(ran3() * ran3()) / 2.;
                    // }
                    // _speed = -_params.vmean * log(ran3());
                    _speed = _params.vmin + _params.vmean * (-log(ran3() * ran3() * ran3())) / 3.;
                }
                // } while (_speed > _params.vcut); // speed
                _speed = std::min(_speed, _params.vcut);
                _speed = std::max(_speed, _params.vmin);

                float dtau = _params.taumean - _params.taumin;
                _tau = _params.taumin - dtau * 0.5 * log(ran3() * ran3());

                // cognitive noise
                float gauss = std::sqrt(-2. * log(ran3())) * cos(2 * M_PI * ran3());
                phi_new = _pose_in_cm.pose.rpy.yaw + dphi_int + _params.gamma_rand * gauss * (1. - _params.alpha_w * fw);
                float dl = _speed * _params.tau0 * (1. - std::exp(-_tau / _params.tau0)) + _params.dc;
                float x_new = _pose_in_cm.pose.xyz.x + dl * std::cos(phi_new);
                float y_new = _pose_in_cm.pose.xyz.y + dl * std::sin(phi_new);
                r_new = std::sqrt(x_new * x_new + y_new * y_new);

                if (std::cos(std::get<1>(state)[cn_idx]) > 0.9999 && std::get<0>(state)[cn_idx] < 3.) {
                    _speed /= 2.;
                }

                _traj_pose.rpy.yaw = _angle_to_pipi(phi_new);

                if (r_new > _params.radius) {
                    _tau = 0;
                    return false;
                }
                else {
                    return true;
                }
            }

            state_t _compute_state() const
            {
                size_t num_fish = _individual_poses.size();

                Eigen::VectorXd distances(num_fish);
                Eigen::VectorXd psis_ij(num_fish);
                Eigen::VectorXd psis_ji(num_fish);
                Eigen::VectorXd dphis(num_fish);

                for (uint i = 0; i < num_fish; ++i) {
                    distances(i) = std::sqrt(
                        std::pow(_pose_in_cm.pose.xyz.x - _individual_poses[i].pose.xyz.x, 2)
                        + std::pow(_pose_in_cm.pose.xyz.y - _individual_poses[i].pose.xyz.y, 2));

                    psis_ij(i) = std::atan2(_individual_poses[i].pose.xyz.y - _pose_in_cm.pose.xyz.y,
                                     _individual_poses[i].pose.xyz.x - _pose_in_cm.pose.xyz.x)
                        - _pose_in_cm.pose.rpy.yaw;

                    psis_ji(i) = std::atan2(_pose_in_cm.pose.xyz.y - _individual_poses[i].pose.xyz.y,
                                     _pose_in_cm.pose.xyz.x - _individual_poses[i].pose.xyz.x)
                        - _individual_poses[i].pose.rpy.yaw;

                    dphis(i) = _individual_poses[i].pose.rpy.yaw - _pose_in_cm.pose.rpy.yaw;
                }

                return {distances, psis_ij, psis_ji, dphis};
            }

            std::tuple<float, float> _compute_interactions(const state_t& state, std::vector<int> neighs)
            {

                /**
                 *
                 * Compute interactions, i.e., attraction/repulsion between the current individual and its neighbours.
                 *
                 **/

                float theta = std::atan2(_pose_in_cm.pose.xyz.y, _pose_in_cm.pose.xyz.x);
                float current_radius_corrected = std::min(static_cast<float>(std::sqrt(_pose_in_cm.pose.xyz.x * _pose_in_cm.pose.xyz.x + _pose_in_cm.pose.xyz.y * _pose_in_cm.pose.xyz.y)), _params.radius);
                float rw = _params.radius - current_radius_corrected;

                float dphi_fish = 0.;
                float dphiw = 0.;
                float fw;

                _traj_pose.rpy.yaw = _pose_in_cm.pose.rpy.yaw;

                for (int j : neighs) {
                    auto neigh = _individual_poses[j];

                    float dij = std::get<0>(state)[j];
                    float psi_ij = abs(_angle_to_pipi(std::get<1>(state)[j]));
                    float psi_ji = abs(_angle_to_pipi(std::get<2>(state)[j]));
                    float dphi_ij = abs(_angle_to_pipi(std::get<3>(state)[j]));

                    // !!! fix this
                    // if (
                    //     neighs.size() == 1 && _params.iuturn
                    //     && psi_ij < _params.psi_c
                    //     && psi_ji < _params.psi_c
                    //     && dij < _params.duturn) {

                    //     ++_num_uturn;
                    //     _pose.yaw = neigh->kick_pose().yaw;
                    // }

                    // wall interaction
                    float theta_w = _angle_to_pipi(_pose_in_cm.pose.rpy.yaw - theta);
                    fw = std::exp(-std::pow(rw / _params.dw, 2));
                    float ow = std::sin(theta_w) * (1. + 0.7 * std::cos(2. * theta_w));
                    dphiw = _params.gamma_wall * fw * ow;

                    float dphiwsym = _params.gamma_sym * std::sin(2. * theta_w) * std::exp(-rw / _params.dw) * rw / _params.dw;
                    float dphiwasym = _params.gamma_asym * std::cos(theta_w) * std::exp(-rw / _params.dw) * rw / _params.dw;

                    dphiw += dphiwsym + dphiwasym;

                    // fish interaction
                    for (int j : neighs) {
                        float dij = std::get<0>(state)[j];
                        float psi_ij = std::get<1>(state)[j];
                        float dphi_ij = std::get<3>(state)[j];

                        float fatt = (dij - 6.) / 3. / (1. + std::pow(dij / 20., 2));
                        // float oatt = std::sin(psi_ij) * (1. - 0.33 * std::cos(psi_ij));
                        // float eatt = 1. - 0.48 * std::cos(dphi_ij) - 0.31 * std::cos(2. * dphi_ij);
                        float oatt = std::sin(psi_ij) * (1. + std::cos(psi_ij));
                        float eatt = 1.;
                        float dphiatt = _params.gamma_attraction * fatt * oatt * eatt;

                        // float fali = (dij + 3. ) / 6. * std::exp(-std::pow(dij / 20., 2));
                        float fali = std::exp(-std::pow(dij / 20., 2));
                        float oali = std::sin(dphi_ij) * (1. + 0.3 * std::cos(2. * dphi_ij));
                        float eali = 1. + 0.6 * std::cos(psi_ij) - 0.32 * std::cos(2. * psi_ij);
                        float dphiali = _params.gamma_alignment * fali * oali * eali;

                        dphi_fish += dphiatt + dphiali;
                    }
                } // for each neighbour
                float dphi_int = dphiw + dphi_fish;

                return {dphi_int, fw};
            }

            std::vector<int>
            _sort_neighbours(
                const Eigen::VectorXd& values, const int kicker_idx, Order order) const
            {
                std::vector<int> neigh_idcs;
                for (int i = 0; i < values.rows(); ++i) {
                    if (i == kicker_idx)
                        continue;
                    neigh_idcs.push_back(i);
                }

                std::sort(std::begin(neigh_idcs), std::end(neigh_idcs), [&](int lhs, int rhs) {
                    return (order == Order::INCREASING) ? (values(lhs) < values(rhs))
                                                        : (values(lhs) > values(rhs));
                });

                return neigh_idcs;
            }

            defaults::RummyIndividualParams _params;

            uint64_t _num_jumps;
            uint64_t _num_kicks;
            uint64_t _num_uturn;

            int _iters;
            double _total_time;
            float _t0;
            float _tau;
            bobi_msgs::Pose _traj_pose;
            float _speed;
        }; // namespace behaviour

    } // namespace behaviour
} // namespace bobi

#endif