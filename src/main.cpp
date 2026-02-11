#include "color_detection.h"
#include "image_pre_process.h"
#include "ros_inter.h"

int main(int argc, char* argv[]){
    ros::init(argc, argv, "scanland");
    ros::NodeHandle nh_image;
    image_transport::ImageTransport it_(nh_image);

    image_convrt(it_);

    ros::Rate loop_rate(30);  // 30Hz
    while(true){
        ros::spinOnce(); 
        // 检查图像是否有效
        if (!image_origin.empty() && image_origin.cols > 0 && image_origin.rows > 0) {
            cv::imshow("img from ros", image_origin);
        }
        cv::waitKey(1);
        loop_rate.sleep();
    }
}