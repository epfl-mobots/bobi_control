#include <ros/ros.h>
#include <bobi_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "eights_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    float radius1, radius2;
    std::vector<float> center;
    float waypoint_angle;
    int rate;
    nh->param<float>("eights/radius1", radius1, 0.08);
    nh->param<float>("eights/radius2", radius2, 0.08);
    nh->param<std::vector<float>>("eights/center", center, {0., 0.});
    nh->param<float>("eights/waypoint_angle", waypoint_angle, 0.26);
    nh->param<int>("eights/rate", rate, 10);

    ros::Publisher wp_pub = nh->advertise<bobi_msgs::PoseStamped>("target_position", 1);

    int num_waypoints = static_cast<int>(2. * M_PI / waypoint_angle);
    std::vector<bobi_msgs::PoseStamped> waypoints;
    for (int i = 0; i < num_waypoints; ++i) {
        bobi_msgs::PoseStamped wp;
        wp.pose.xyz.x = radius1 * std::cos(waypoint_angle * i + M_PI / 2) + center[0];
        wp.pose.xyz.y = radius1 * std::sin(waypoint_angle * i + M_PI / 2) + (center[1] - radius1);
        waypoints.push_back(wp);
    }

    for (int i = 0; i < num_waypoints; ++i) {
        bobi_msgs::PoseStamped wp;
        wp.pose.xyz.x = radius2 * std::cos(-waypoint_angle * i + 3 * M_PI / 2) + center[0];
        wp.pose.xyz.y = radius2 * std::sin(-waypoint_angle * i + 3 * M_PI / 2) + (center[1] + radius2);
        waypoints.push_back(wp);
    }

    ros::Rate loop_rate(rate);
    int iter = 0;
    while (ros::ok()) {
        ROS_INFO("(x=%f, y=%f)", waypoints[iter].pose.xyz.x, waypoints[iter].pose.xyz.y);
        wp_pub.publish(waypoints[iter++]);
        if (iter == num_waypoints * 2) {
            iter = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}