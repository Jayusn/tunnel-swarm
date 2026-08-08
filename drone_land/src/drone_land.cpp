#include <ros/ros.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// 回调函数更新无人机状态
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_land");
    ros::NodeHandle nh;

    // 订阅无人机的状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    // 服务客户端请求降落
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    // 服务客户端请求切换飞行模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    // 等待FCU连接
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        if (current_state.mode != "AUTO.LAND" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent) {
                ROS_INFO("Land enabled");
            }
            last_request = ros::Time::now();
        } else {
            mavros_msgs::CommandTOL land_cmd;
            land_cmd.request.altitude = 0;
            land_cmd.request.latitude = 0;
            land_cmd.request.longitude = 0;
            land_cmd.request.min_pitch = 0;
            land_cmd.request.yaw = 0;

            if (land_client.call(land_cmd) && land_cmd.response.success) {
                ROS_INFO("Landing...");
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
