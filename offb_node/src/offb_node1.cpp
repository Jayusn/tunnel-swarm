//这一版本起飞后一直向上飞
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

// 回调函数更新无人机状态
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// 回调函数订阅遥控器输入
mavros_msgs::RCIn rc;
int rc_value;
void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg) {
    rc = *msg;
    rc_value = rc.channels[6]; // 订阅第七通道
    if (rc_value == 0) { // 检测遥控器第七通道值为0时执行
        ROS_INFO("Emergency cut off");

        ros::NodeHandle nh;
        // 初始化 SetMode 服务客户端
        ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        // 切换到MANUAL模式
        mavros_msgs::SetMode manual_set_mode;
        manual_set_mode.request.custom_mode = "MANUAL";
        if (set_mode_client.call(manual_set_mode) && manual_set_mode.response.mode_sent) {
            ROS_INFO("Switched to MANUAL mode");
        } else {
            ROS_ERROR("Failed to switch to MANUAL mode");
        }

        // 等待一段时间以确保模式切换完成
        ros::Duration(1.0).sleep();

        // 发布紧急断电消息
        ros::Publisher pub = nh.advertise<std_msgs::Empty>("/emergency_stop", 10);
        std_msgs::Empty empty_msg;
        pub.publish(empty_msg);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // 订阅无人机的状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    // 订阅遥控器的输入
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 10, rc_cb);
    
    // 发布位置指令消息
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    // 发布悬停完成标志消息
    ros::Publisher hover_done_pub = nh.advertise<std_msgs::Bool>("hover_done", 10);

    // 服务客户端请求解锁无人机
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    
    // 服务客户端请求切换飞行模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 设置发布频率
    ros::Rate rate(20.0);
    
    // 等待FCU连接
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // 设置初始位置
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.7;

    // 发送一组初始位置指令
    for(int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 请求切换模式消息，设置为OFFBOARD
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 请求解锁消息，设置为true
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 记录上次请求的时间
    ros::Time last_request = ros::Time::now();

    bool takeoff = false;
    bool offboard_mode_enabled = false;
    int offboard_flag = 0;

    while(ros::ok()) {
        if(!takeoff) {
            if(current_state.mode != "OFFBOARD" && !offboard_mode_enabled &&
               (ros::Time::now() - last_request > ros::Duration(2.0)) && (offboard_flag == 0)) {
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                    offboard_mode_enabled = true;
                }
                last_request = ros::Time::now();
            } else {
                if(!current_state.armed &&
                   (ros::Time::now() - last_request > ros::Duration(2.0)) && (offboard_flag == 0)) {
                    if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
            if(current_state.armed && 
               (ros::Time::now() - last_request > ros::Duration(5.0))) {
                takeoff = true;
                ROS_INFO("Vehicle stabilized");
                ROS_INFO("Hovering...");
                last_request = ros::Time::now();
                
                // 发布悬停完成标志消息
                std_msgs::Bool hover_done_msg;
                hover_done_msg.data = true;
                hover_done_pub.publish(hover_done_msg);

                // 停止发送位置指令
                break;
            }
        }

        if(current_state.mode != "OFFBOARD" &&
           (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if(!current_state.armed &&
               (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // 不断发送位置指令以保持悬停
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
