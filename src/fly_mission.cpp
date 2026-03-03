#include "collision_avoidance.h"
// 从此开始---0
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PointStamped.h>
#include <vision_msgs/Detection2DArray.h>
/// 到此结束---

// 全局变量定义
int mission_num       = 0;
float if_debug        = 0;
float err_max         = 0.2;
// 从此开始---1
int yolo_detect_mode  = 0;
bool search_mode_flag = false;
bool yolo_detect_flag = false;
float yolo_follow_kp  = 1;
float yolo_vel[2];
float time_threshold = 1;
std::string takeoff_color;
std::string land_color;
bool land_detected;
std_msgs::String takeoff_msg;
std_msgs::String land_color_msg;
std_msgs::Bool land_detected_msg;
std_msgs::Int8 mission_num_msg;
geometry_msgs::PointStamped yolo_result;
vision_msgs::Detection2DArray latest_detections;
enum { search_mode, follow_mode, holding_mode };
// 到此结束---
void print_param() {
    std::cout << "=== 控制参数 ===" << std::endl;
    std::cout << "err_max: " << err_max << std::endl;
    std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
    std::cout << "if_debug: " << if_debug << std::endl;
    std::cout << "p_xy: " << p_xy << std::endl;
    if (if_debug == 1)
        cout << "自动offboard" << std::endl;
    else
        cout << "遥控器offboard" << std::endl;
}
// 从此开始---2
void yolo_result_cb(const geometry_msgs::PointStamped::ConstPtr &msg) {
    yolo_result = *msg;
}

void yolo_boxes_cb(const vision_msgs::Detection2DArray::ConstPtr &msg) {
    latest_detections = *msg;
    // 可以进一步处理，例如找出最大框、特定类别等
}

void takeoff_cb(const std_msgs::String::ConstPtr &msg) {
    takeoff_msg   = *msg;
    takeoff_color = takeoff_msg.data;
}

void land_color_cb(const std_msgs::String::ConstPtr &msg) {
    land_color_msg = *msg;
    land_color     = land_color_msg.data;
}

void land_detected_cb(const std_msgs::Bool::ConstPtr &msg) {
    land_detected_msg = *msg;
    land_detected     = land_detected_msg.data;
}
// 到此结束---

int main(int argc, char **argv) {
    // 防止中文输出乱码
    setlocale(LC_ALL, "");

    // 初始化ROS节点
    ros::init(argc, argv, "collision_avoidance");
    ros::NodeHandle nh;

    // 订阅mavros相关话题
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub =
        nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

    // 发布无人机多维控制话题
    ros::Publisher mavros_setpoint_pos_pub =
        nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);
    ros::Subscriber livox_sub =
        nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10, livox_custom_cb);

    // 创建服务客户端
    ros::ServiceClient arming_client =
        nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient ctrl_pwm_client =
        nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
    // 从此开始---3
    // yolo result sub
    ros::Subscriber yolo_sub =
        nh.subscribe<geometry_msgs::PointStamped>("/yolo/detection", 10, yolo_result_cb);

    // takeoff color sub
    ros::Subscriber takeoff_color_sub =
        nh.subscribe<std_msgs::String>("/color_detect/takeoff_color", 10, takeoff_cb);

    // land color sub
    ros::Subscriber land_color_sub =
        nh.subscribe<std_msgs::String>("/color_detect/land_color", 10, land_color_cb);

    // land detected sub
    ros::Subscriber land_detected_sub =
        nh.subscribe<std_msgs::Bool>("/color_detect/land_detected", 10, land_detected_cb);

    // mission_num pub
    ros::Publisher mission_num_pub = nh.advertise<std_msgs::Int8>("/color_detect/mission_num", 10);
    // 到此结束---
    //  设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
    ros::Rate rate(20);

    // 参数读取

    nh.param<float>("err_max", err_max, 0);
    nh.param<float>("if_debug", if_debug, 0);
    nh.param<double>("zero_plane_height", zero_plane_height, 0);
    nh.param<double>("height_threshold", height_threshold, 0.05);
    // nh.param<double>("min_range", min_range, 0.1);
    // nh.param<double>("max_range", max_range, 30.0);
    nh.param<int>("num_bins", num_bins, 360);

    // nh.param<float>("R_outside", R_outside, 0.0);
    // nh.param<float>("R_inside", R_inside, 0.0);
    // nh.param<float>("p_R", p_R, 0.0);
    // nh.param<float>("p_r", p_r, 0.0);
    // nh.param<float>("vel_collision_max", vel_collision_max, 0.0);
    // nh.param<float>("vel_sp_max", vel_sp_max, 0.0);
    // 从此开始---4
    nh.param<float>("p_xy", p_xy, 0.0);
    nh.param<float>("vel_track_max", vel_track_max, 0.0);

    nh.param<float>("yolo_follow_kp", yolo_follow_kp, 1);
    nh.param<float>("time_threshold", time_threshold, 1);
    // 到此结束---
    print_param();

    int choice = 0;
    std::cout << "1 to go on , else to quit" << std::endl;
    std::cin >> choice;
    if (choice != 1) return 0;
    ros::spinOnce();
    rate.sleep();

    // 等待连接到飞控
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    // 设置无人机的期望位置

    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x       = 0;
    setpoint_raw.position.y       = 0;
    setpoint_raw.position.z       = ALTITUDE;
    setpoint_raw.yaw              = 0;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "ok" << std::endl;

    // 定义客户端变量，设置为offboard模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 定义客户端变量，请求无人机解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value  = true;

    // 记录当前时间，并赋值给变量last_request
    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if (if_debug == 1) {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                }
            }
            else { ROS_INFO("Waiting for OFFBOARD mode"); }
            last_request = ros::Time::now();
        }
        else {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        // 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
        if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2) {
            if (ros::Time::now() - last_request > ros::Duration(1.0)) {
                mission_num          = 1;
                last_request         = ros::Time::now();
                // 从此开始---5
                mission_num_msg.data = 1;
                mission_num_pub.publish(mission_num_msg);
                // 到此结束---
                break;
            }
        }

        mission_pos_cruise(0, 0, ALTITUDE, 0, err_max);
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        ROS_WARN("mission_num = %d", mission_num);

        switch (mission_num) {
        // mission1: 起飞
        case 1:
            if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max)) {
                mission_num  = 2;
                last_request = ros::Time::now();
            }
            else if (ros::Time::now() - last_request >= ros::Duration(3.0)) {
                mission_num  = 2;
                last_request = ros::Time::now();
            }
            break;

        // 想办法到(6,4)
        case 2:
            if (control_by_vel(6.0, 0.0, ALTITUDE, 0, err_max)) {
                mission_num      = 3;
                last_request     = ros::Time::now();
                yolo_detect_mode = search_mode;
            }
            break;

            // scan_land
        case 3: {
            // 从此开始---6
            mission_num_msg.data = 2;
            mission_num_pub.publish(mission_num_msg);
            ROS_INFO("yolo result: x: %f ,y: %f ,is_detected: %d", yolo_result.point.x,
                     yolo_result.point.y, yolo_result.point.z);
            switch (yolo_detect_mode) {
            case search_mode:
                ROS_INFO("search mode");
                if (land_detected && land_color.compare(takeoff_color) == 0 && yolo_result.point.z)
                {
                    yolo_detect_mode = follow_mode;
                    last_request     = ros::Time::now();
                    break;
                }
                if (!search_mode_flag && control_by_vel(6.0, 4.0, ALTITUDE, 0, err_max)) {
                    search_mode_flag = true;
                }
                else if (search_mode_flag && control_by_vel(6.0, 0.0, ALTITUDE, 0, err_max)) {
                    search_mode_flag = false;
                }
                break;
            case follow_mode:
                if (ros::Time::now() - last_request > ros::Duration(time_threshold) &&
                        land_detected && yolo_result.point.z == false ||
                    land_color.compare(takeoff_color))
                {
                    yolo_detect_mode = search_mode;
                    last_request     = ros::Time::now();
                    break;
                }
                ROS_INFO("follow mode");
                if (land_color.compare(takeoff_color) == 0 &&
                    hypot(yolo_result.point.x, yolo_result.point.y) < 0.1)
                {
                    // yolo_detect_mode = holding_mode;
                    // 此处进入降落任务
                    mission_num  = 4;
                    last_request = ros::Time::now();
                    break;
                }
                yolo_vel[0] = yolo_result.point.y * yolo_follow_kp;
                yolo_vel[1] = yolo_result.point.x * yolo_follow_kp;
                for (int i = 0; i < 2; i++) { yolo_vel[i] = satfunc(yolo_vel[i], err_max); }
                ROS_WARN("Velocity Command Body before CA: vx: %.2f , vy: %.2f ", yolo_vel[0],
                         yolo_vel[1]);
                setpoint_raw.type_mask =
                    1 + 2 /*+ 4  +8 + 16 */ + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
                setpoint_raw.velocity.x = yolo_vel[0];
                setpoint_raw.velocity.y = yolo_vel[1];
                setpoint_raw.position.z = ALTITUDE;
                setpoint_raw.yaw        = 0;
                break;
            }
            // 到此结束---
            break;
        }
        // 降落
        case 4:
            // 从此开始---7
            mission_num_msg.data = 3;
            mission_num_pub.publish(mission_num_msg);
            // 到此结束---
            if (precision_land()) {
                mission_num  = -1;  // 任务结束
                last_request = ros::Time::now();
            }
            break;
        }
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();

        if (mission_num == -1) { exit(0); }
    }
    return 0;
}

