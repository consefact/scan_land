// yolo_detector_node.cpp
#include "yolo_detector.h"
#include <cmath>
#include <algorithm>
// #include <fstream>
// #include <chrono>

YOLODetector::YOLODetector(ros::NodeHandle& nh)
    : nh_(nh),
      input_width_(640),
      input_height_(640),
      confidence_threshold_(0.5),
      nms_threshold_(0.4),
      target_detected(false),
      num_classes_from_config(1)
    // , last_process_time_(0.0)
    // , frame_count_(0) 
    {
}

YOLODetector::~YOLODetector() = default;

bool YOLODetector::initialize() {
    ROS_INFO("初始化YOLO检测节点...");
    // 加载参数
    nh_.param<std::string>("model_path", model_path_, "");
    nh_.param<float>("confidence_threshold", confidence_threshold_, 0.5);
    nh_.param<float>("nms_threshold", nms_threshold_, 0.4);
    nh_.param<int>("input_width", input_width_, 640);
    nh_.param<int>("input_height", input_height_, 640);
    nh_.param<int>("num_classes_from_config", num_classes_from_config, 1);

    ROS_INFO("input_width: %d, input_height: %d", input_width_, input_height_);
    
    if (model_path_.empty()) {
        ROS_ERROR("未指定模型路径");
        return false;
    }
    
    // 加载模型
    try {
        loadModel(model_path_);
    } catch (const cv::Exception& e) {
        ROS_ERROR("加载模型失败: %s", e.what());
        return false;
    }
    
    // 初始化ROS通信
    image_transport::ImageTransport it(nh_);
    image_sub_ = it.subscribe("/camera/image_raw", 1, 
                              &YOLODetector::imageCallback, this);
    detection_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/yolo/detection", 10);
    debug_image_pub_ = it.advertise("/yolo/debug_image", 1);
    
    ROS_INFO("YOLO检测节点初始化完成");
    return true;
}

void YOLODetector::loadModel(const std::string& model_path) {
    ROS_INFO("加载模型: %s", model_path.c_str());
    
    if (model_path.find(".onnx") == std::string::npos) {
        ROS_ERROR("modle file is not the kind like \".onnx\"");
        return;
    }
    net_ = cv::dnn::readNetFromONNX(model_path);
    // 设置计算后端（优先使用CUDA）
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    
    ROS_INFO("模型加载成功");
}

void YOLODetector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // auto start = std::chrono::high_resolution_clock::now();
    
    try {
        // 转换ROS图像到OpenCV
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        ROS_INFO("cv_bridge convert complete");
        cv::Mat image = cv_ptr->image.clone();
        ROS_INFO("image width: %d, image height: %d", image.cols, image.rows);
        // 执行检测
        std::vector<cv::Rect> detections = detect(image);
        ROS_INFO("step detect complete");
        // 发布结果
        publishResults(detections, msg->header.stamp);
        ROS_INFO("step result pub complete");
        // 发布调试图像
        if (debug_image_pub_.getNumSubscribers() > 0) {
            cv::Mat debug_img = image.clone();
            for (const auto& rect : detections) {
                cv::rectangle(debug_img, rect, cv::Scalar(0, 255, 0), 2);
            }
            
            sensor_msgs::ImagePtr debug_msg = cv_bridge::CvImage(
                msg->header, "bgr8", debug_img).toImageMsg();
            debug_image_pub_.publish(debug_msg);
        }
        ROS_INFO("step debug pub");
        
    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("图像转换错误: %s", e.what());
    } catch (const cv::Exception& e) {
        ROS_ERROR("OpenCV异常: %s", e.what());
    } catch (const std::exception& e) {
        ROS_ERROR("标准异常: %s", e.what());
    } catch (...) {
        ROS_ERROR("未知异常");
    }
    
    // auto end = std::chrono::high_resolution_clock::now();
    // last_process_time_ = std::chrono::duration<double, std::milli>(end - start).count();
    
    // frame_count_++;
    // if (frame_count_ % 30 == 0) {
    //     ROS_INFO("处理时间: %.2f ms", last_process_time_);
    // }
}

// --- 新增辅助函数 ---
// Sigmoid function for confidence calculation
static float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

// IOU calculation
static float calculateIOU(const cv::Rect& box1, const cv::Rect& box2) {
    int x1 = std::max(box1.x, box2.x);
    int y1 = std::max(box1.y, box2.y);
    int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
    int y2 = std::min(box1.y + box1.height, box2.y + box2.height);

    float intersection_area = std::max(0.f, static_cast<float>(x2 - x1)) *
                              std::max(0.f, static_cast<float>(y2 - y1));
    float union_area = box1.area() + box2.area() - intersection_area;

    if (union_area == 0) {
        return 0;
    }
    return intersection_area / union_area;
}
// --- 结束新增辅助函数 ---

std::vector<cv::Rect> YOLODetector::detect(const cv::Mat& image) {
    std::vector<cv::Rect> detections;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    ROS_INFO_STREAM("进入detect函数，输入图像尺寸: " << image.cols << "x" << image.rows);

    // 1. 预处理图像
    cv::Mat blob = cv::dnn::blobFromImage(image, 1.0/255.0,
                                          cv::Size(input_width_, input_height_), // Use the loaded params
                                          cv::Scalar(), true, false);
    ROS_INFO_STREAM("Blob created. Shape: [" << blob.size[0] << ", " << blob.size[1] << ", " << blob.size[2] << ", " << blob.size[3] << "]");

    // 2. 设置输入
    net_.setInput(blob);
    ROS_INFO("输入已设置到网络");

    // 3. 前向推理
    std::vector<cv::Mat> outputs;
    try {
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());
        ROS_INFO("模型前向推理完成，输出层数量: %zu", outputs.size());
    } catch (const cv::Exception& e) {
         ROS_ERROR("模型前向推理失败: %s", e.what());
         return detections; // 推理失败，返回空结果
    }

    // --- 添加这行来确认是否进入解析阶段 ---
    ROS_INFO("开始解析模型输出...");

    if (outputs.empty()) {
        ROS_WARN("模型前向推理成功但没有输出。");
        return detections; // Return empty vector if no output
    }

    cv::Mat output = outputs[0]; // Get the first (and usually only) output layer
    ROS_INFO("Raw model output shape: [%d, %d, %d]", output.size[0], output.size[1], output.size[2]); // This is the line that should print if we reach here
    ROS_INFO("Raw model output dims: %d, total elements: %ld", output.dims, output.total());

    // --- YOLOv8 Output Parsing ---
    // YOLOv8's output shape is typically either [batch, anchors, (4 + num_classes)]
    // (for channels_last) or [batch, (4 + num_classes), anchors] (for channels_first).
    // We need to determine which one and reshape accordingly.

    const int batch_size = output.size[0];
    const int channels_dim = output.size[1];
    const int anchors_dim = output.size[2];

    cv::Mat reshaped_output;

    // Check for channels_first format: [1, (4 + nc), num_anchors]
    // Common shapes might be [1, 84, 8400] for 80 classes (4 bbox + 80 classes)
    if (output.dims == 3 && channels_dim == (4 + num_classes_from_config)) { // num_classes_from_config needs to be defined/fetched
        // Transpose from [batch, channels, anchors] to [batch, anchors, channels]
        // First, reshape to [channels, anchors] effectively ignoring batch for transposition
        cv::Mat temp_mat = output.reshape(1, {channels_dim, anchors_dim}); // Shape becomes [ch, an]
        cv::transpose(temp_mat, reshaped_output); // Shape becomes [an, ch]
        // Now reshaped_output is [num_anchors, 4 + nc]
    }
    // Check for channels_last format: [1, num_anchors, (4 + num_classes)]
    // Common shapes might be [1, 8400, 84] for 80 classes
    else if (output.dims == 3 && anchors_dim == (4 + num_classes_from_config)) {
        // Already in [batch, anchors, channels], just remove batch dim
        reshaped_output = output.reshape(1, anchors_dim); // Shape becomes [anchors, channels]
    }
    else {
        // If num_classes is unknown at compile time, we can make a guess based on common shapes.
        // YOLOv8 often has 80 classes (COCO). Let's assume 80 for now, adjust if needed.
        const int assumed_num_classes = 1; // <-- CHANGE THIS IF YOUR MODEL HAS DIFFERENT NUM CLASSES!
        const int expected_channels_cfirst = 4 + assumed_num_classes;
        const int expected_channels_clast = 4 + assumed_num_classes;

        if (output.dims == 3 && channels_dim == expected_channels_cfirst) {
            ROS_INFO("Detected channels_first output format.");
            // Reshape and transpose
            cv::Mat temp_mat = output.reshape(1, {channels_dim, anchors_dim});
            cv::transpose(temp_mat, reshaped_output);
        } else if (output.dims == 3 && anchors_dim == expected_channels_clast) {
            ROS_INFO("Detected channels_last output format.");
            // Remove batch dimension
            reshaped_output = output.reshape(1, anchors_dim);
        } else {
            ROS_ERROR("Unexpected output shape after forward pass: [%d, %d, %d]. Expected [1, %d, N] or [1, N, %d]",
                    batch_size, channels_dim, anchors_dim, expected_channels_cfirst, expected_channels_clast);
            // Print raw shape again in case of error
            for (int i = 0; i < output.dims; ++i) {
                printf("  Size[%d] = %d\n", i, output.size[i]);
            }
            return detections;
        }
    }

    const int num_classes = reshaped_output.cols - 4; // Number of classes inferred from reshaped output
    const int num_detections = reshaped_output.rows;  // Number of anchor/prediction boxes

    // Iterate through outputs using the correctly reshaped matrix
    for (int i = 0; i < num_detections; ++i) {
        // Extract class scores [4:] -> from column 4 onwards
        cv::Mat classes_scores = reshaped_output.row(i).colRange(4, reshaped_output.cols); // More robust than Rect

        cv::Point class_id_point;
        double max_class_score;
        cv::minMaxLoc(classes_scores, nullptr, &max_class_score, nullptr, &class_id_point);
        float confidence = sigmoid(max_class_score);

        if (confidence >= confidence_threshold_) {
            // Extract box coordinates [cx, cy, w, h] (columns 0-3)
            float cx = reshaped_output.at<float>(i, 0);
            float cy = reshaped_output.at<float>(i, 1);
            float w = reshaped_output.at<float>(i, 2);
            float h = reshaped_output.at<float>(i, 3);

            // Convert from normalized coordinates to absolute coordinates
            int x = static_cast<int>((cx - 0.5 * w) * image.cols);
            int y = static_cast<int>((cy - 0.5 * h) * image.rows);
            int width = static_cast<int>(w * image.cols);
            int height = static_cast<int>(h * image.rows);

            x = std::max(0, x);
            y = std::max(0, y);
            width = std::min(image.cols - x, width);
            height = std::min(image.rows - y, height);

            if (width > 0 && height > 0) {
                detections.push_back(cv::Rect(x, y, width, height));
                confidences.push_back(confidence);
                class_ids.push_back(class_id_point.x);
            }
        }
    }

    // --- NMS ---
    std::vector<cv::Rect> nms_boxes = detections;
    std::vector<float> nms_confidences = confidences;
    std::vector<int> indices;
    cv::dnn::NMSBoxes(nms_boxes, nms_confidences,
                    confidence_threshold_, nms_threshold_, indices);

    std::vector<cv::Rect> final_detections;
    for (int idx : indices) {
        final_detections.push_back(detections[idx]); // Use original detections
    }

    return final_detections;
    // --- End YOLOv8 Output Parsing ---
}

void YOLODetector::publishResults(const std::vector<cv::Rect>& detections,
                                     const ros::Time& stamp) {
    if (detections.empty()){
        target_detected = false;
        return;
    }
    target_detected = true;
    
    // 选择最大的检测框（假设是H地块）
    int max_area_idx = 0;
    double max_area = 0;
    for (size_t i = 0; i < detections.size(); i++) {
        double area = detections[i].area();
        if (area > max_area) {
            max_area = area;
            max_area_idx = i;
        }
    }
    
    // 发布目标位置（图像中心归一化坐标）
    geometry_msgs::PointStamped target_msg;
    target_msg.header.stamp = stamp;
    target_msg.header.frame_id = "downward_camera";
    
    // 假设图像大小为640x480（需要从实际图像获取）
    int img_width = 640;
    int img_height = 480;
    
    cv::Rect target = detections[max_area_idx];
    target_msg.point.x = (target.x + target.width/2 - img_width/2.0) / img_width;
    target_msg.point.y = (target.y + target.height/2 - img_height/2.0) / img_height;
    target_msg.point.z = 0.0;

    target_info.x = target_msg.point.x;
    target_info.y = target_msg.point.y;
    
    detection_pub_.publish(target_msg);
    
    // // 发布可视化标记
    // publishMarkers(detections, stamp);
}

// void YOLODetector::publishMarkers(const std::vector<cv::Rect>& detections,
//                                      const ros::Time& stamp) {
//     visualization_msgs::Marker marker;
//     marker.header.stamp = stamp;
//     marker.header.frame_id = "downward_camera";
//     marker.ns = "yolo_detection";
//     marker.id = 0;
//     marker.type = visualization_msgs::Marker::CUBE;
//     marker.action = visualization_msgs::Marker::ADD;
    
//     // 设置位置（假设在相机前方2米处）
//     marker.pose.position.x = 2.0;
//     marker.pose.position.z = 0.5;
//     marker.pose.orientation.w = 1.0;
    
//     // 设置大小
//     marker.scale.x = 0.5;
//     marker.scale.y = 0.5;
//     marker.scale.z = 0.1;
    
//     // 设置颜色（绿色）
//     marker.color.r = 0.0;
//     marker.color.g = 1.0;
//     marker.color.b = 0.0;
//     marker.color.a = 0.5;
    
//     marker.lifetime = ros::Duration(0.1);
//     marker_pub_.publish(marker);
// }

void YOLODetector::run() {
    ROS_INFO("YOLO检测节点开始运行");
    ros::spin();
}

bool YOLODetector::getResult_detected(){
    return target_detected;
}

geometry_msgs::Point YOLODetector::getResult_info(){
    return target_info;
}