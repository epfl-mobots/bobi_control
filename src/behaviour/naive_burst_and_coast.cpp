#include <ros/ros.h>

#include <bobi_control/behaviour/naive_burst_and_coast.hpp>

using namespace bobi;
using namespace behaviour;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "naive_burst_and_coast_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    int id;
    nh->param<int>("robot_id", id, -1);

    std::string pose_topic;
    nh->param<std::string>("pose_topic", pose_topic, "naive_poses");

    float wheel_radius;
    nh->param<float>("wheel_radius", wheel_radius, -1);

    float wheel_distance;
    nh->param<float>("wheel_distance", wheel_distance, -1);

    assert(wheel_radius > 0);
    assert(wheel_distance > 0);

    ROS_INFO("Robot %d controller using %s topic for pose info", id, pose_topic.c_str());

    NaiveBurstAndCoast ctrl(nh, id, pose_topic, wheel_radius, wheel_distance);

    double rate;
    nh->param<double>("naive_burst_and_coast/rate", rate, 30);
    ros::Rate loop_rate(rate);
    while (ros::ok()) {
        ctrl.spin_once();
        ros::spinOnce();
        // if (ctrl.kicked()) {
        //     ros::Duration(ctrl.get_tau()).sleep();
        // } else {
        loop_rate.sleep();
        // }
    }

    return 0;
}