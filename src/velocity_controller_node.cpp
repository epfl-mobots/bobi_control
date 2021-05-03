#include <ros/ros.h>

#include <bobi_control/velocity_control.hpp>

using namespace bobi;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_controller_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    VelocityControl ctrl(nh);

    int rate;
    nh->param<int>("rate", rate, 30);
    ros::Rate loop_rate(rate);
    while (ros::ok()) {
        ctrl.spin_once();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}