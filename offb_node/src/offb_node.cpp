//在原版的基础上融合了超维的高度和时间判断，确保无人机在切入OFFBOARD模式和解锁后能安全操作。
//这个是进一步的改进版本

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Empty.h>
#include <cmath>

// 室外宏定义飞行高度2米，室内可适当降低高度
#define ALTITUDE  1.2

// 回调函数更新无人机状态
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// 回调函数订阅遥控器输入
mavros_msgs::RCIn rc;
int rc_value;
bool emergency = false; // 紧急状态标志

void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg) {
    rc = *msg;
    rc_value = rc.channels[6]; // 订阅第七通道
    if (rc_value == 0) { // 检测遥控器第七通道值为0时执行
        emergency = true; // 设置紧急状态标志为true
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
    // 服务客户端请求解锁无人机
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    // 服务客户端请求切换飞行模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 设置发布频率
    ros::Rate rate(20.0);

    // 等待FCU连接
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // 设置初始位置
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = ALTITUDE;

    // 发送一组初始位置指令，确保无人机能稳定在该位置
    for (int i = 100; ros::ok() && i > 0; --i) {
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
    bool takeoff = false; // 起飞标志
    bool offboard_mode_enabled = false; // OFFBOARD模式启用标志

    while (ros::ok()) {
        // 检查紧急状态
        if (emergency) {
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
            break; // 退出主循环
        }

        // 如果未起飞，尝试切换模式和解锁
        if (!takeoff) {
            // 切换到OFFBOARD模式
            if (current_state.mode != "OFFBOARD" && !offboard_mode_enabled && (ros::Time::now() - last_request > ros::Duration(2.0))) {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                    offboard_mode_enabled = true;
                }
                last_request = ros::Time::now();
            } 
            // 解锁无人机
            else if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(2.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }

            // 如果无人机已经解锁且稳定，标记起飞完成
            if (current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                takeoff = true;
                ROS_INFO("Vehicle stabilized");
                ROS_INFO("Tracking started...");
                last_request = ros::Time::now();
            }
        }

        // 再次检查OFFBOARD模式和解锁状态
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        // 判断无人机高度和时间，确保在安全操作范围内
        if (fabs(pose.pose.position.z - ALTITUDE) < 0.5 && (ros::Time::now() - last_request > ros::Duration(3.0))) {
            break;
        }

        // 添加时间判断，使得无人机跳出模式切换循环
        if (ros::Time::now() - last_request > ros::Duration(8.0)) {
            break;
        }

        // 不断发送位置指令
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    // 主循环，发布期望位置
    while (ros::ok()) {
        local_pos_pub.publish(pose);
        ros::spinOnce();

        // 配合rate(20.0)使用，使得while按照20Hz的频率执行
        rate.sleep();
    }

    return 0;
}
