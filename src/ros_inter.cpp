#include "ros_inter.h"

cv::Mat image_origin;
std::mutex image_mutex;

void image_convrt(image_transport::ImageTransport& it_){

    static image_transport::Subscriber image_sub_;

    image_sub_ = it_.subscribe("/camera/image_raw", 1, &image_callback);
}

void image_callback(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    std::lock_guard<std::mutex> lock(image_mutex);
    image_origin = cv_ptr->image.clone();
    ROS_INFO("Received image: %dx%d", image_origin.cols, image_origin.rows);
}