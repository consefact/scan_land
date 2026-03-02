#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from ultralytics import YOLO


class YOLODetector:
    def __init__(self):
        # 初始化节点（可在外部完成，这里为了自包含也允许）
        rospy.init_node('yolo_detector', anonymous=True)
        self.bridge = CvBridge()

        # 加载参数
        self.model_path = rospy.get_param('~model_path', '')
        self.conf_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.nms_threshold = rospy.get_param('~nms_threshold', 0.4)
        self.input_width = rospy.get_param('~input_width', 640)
        self.input_height = rospy.get_param('~input_height', 640)

        if not self.model_path:
            rospy.logerr("未指定模型路径")
            raise ValueError("model_path is required")

        # 加载模型
        try:
            self.model = YOLO(self.model_path, task='detect')
            rospy.loginfo("模型加载成功: %s", self.model_path)
        except Exception as e:
            rospy.logerr("加载模型失败: %s", e)
            raise

        # 状态变量
        self.target_detected = False
        self.target_info = Point()  # 归一化坐标

        # 订阅与发布
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback, queue_size=1)
        self.detection_pub = rospy.Publisher('/yolo/detection', PointStamped, queue_size=10)
        self.debug_image_pub = rospy.Publisher('/yolo/debug_image', Image, queue_size=1)
        self.bbox_pub = rospy.Publisher('/yolo/detection_boxes', Detection2DArray, queue_size=10)
        rospy.loginfo("YOLO检测节点初始化完成")

    def image_callback(self, msg):
        try:
            # 转换ROS图像到OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rospy.logdebug("接收到图像: %dx%d", cv_image.shape[1], cv_image.shape[0])

            # 执行检测
            detections = self.detect(cv_image)
            rospy.loginfo("detections: %i", len(detections))
            # 发布检测结果
            self.publish_results(detections, msg.header.stamp)


            # 发布调试图像（带框）
            if self.debug_image_pub.get_num_connections() > 0:
                debug_img = cv_image.copy()
                for rect in detections:
                    x, y, w, h = rect
                    cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 4)
                # cv2.imshow("debig_image", debug_img)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            rospy.logerr("图像处理异常: %s", e)

    def detect(self, image):
        """
        使用YOLO模型检测图像中的目标
        返回: list of (x, y, width, height) 的矩形列表（像素坐标）
        """
        # 模型推理：设置输入尺寸（可选），应用置信度阈值和IOU阈值
        results = self.model.predict(
            source=image,
            imgsz=(self.input_height, self.input_width),  # 模型期望正方形，但此处也可自适应
            conf=self.conf_threshold,
            iou=self.nms_threshold,
            verbose=False
        )

        detections = []
        if not results or len(results) == 0:
            return detections

        # 取第一个结果（假设只有一张图）
        result = results[0]
        if result.boxes is None:
            return detections

        # boxes.xyxy: [N, 4] 左上右下坐标 (绝对像素)
        boxes = result.boxes.xyxy.cpu().numpy()
        for box in boxes:
            x1, y1, x2, y2 = box.astype(int)
            w = x2 - x1
            h = y2 - y1
            if w > 0 and h > 0:
                detections.append((x1, y1, w, h))
                # rospy.loginfo("bbox: x: %i ,y: %i ,w: %i ,h: %i", x1, y1, w, h)

        return detections

    def publish_results(self, detections, stamp):
        target_msg = PointStamped()
        if not detections:
            self.target_detected = False
            target_msg.point.z = self.target_detected
            self.detection_pub.publish(target_msg)
            return

        self.target_detected = True

        # 选择最大的检测框（假设为目标）
        max_area = 0
        max_rect = None
        for rect in detections:
            x, y, w, h = rect
            area = w * h
            if area > max_area:
                max_area = area
                max_rect = rect

        # 假设图像尺寸（可从实际图像获取，但这里用参数保持一致）
        img_w = self.input_width
        img_h = self.input_height

        x, y, w, h = max_rect
        center_x = x + w // 2
        center_y = y + h // 2

        # 计算归一化坐标（相对于图像中心，范围[-0.5,0.5]）
        norm_x = (center_x - img_w / 2.0) / img_w
        norm_y = (center_y - img_h / 2.0) / img_h

        # 发布PointStamped
        target_msg.header.stamp = stamp
        target_msg.header.frame_id = 'downward_camera'
        target_msg.point.x = norm_x
        target_msg.point.y = norm_y
        target_msg.point.z = self.target_detected
        rospy.loginfo("yolo_result: x: %f, y: %f, is_detected: %i", norm_x, norm_y, self.target_detected)

        self.detection_pub.publish(target_msg)

        # 更新内部状态
        self.target_info.x = norm_x
        self.target_info.y = norm_y

        """
        将检测框列表封装为 Detection2DArray 消息并发布
        detections: list of (x, y, w, h) 像素坐标
        """
        bbox_array = Detection2DArray()
        bbox_array.header = target_msg.header   # 复用图像的时间戳和坐标系（通常为 camera_frame）

        for (x, y, w, h) in detections:
            detection = Detection2D()
            detection.header = bbox_array.header

            # 假设这里没有类别和置信度信息，可留空或填充默认值
            # 若有类别和置信度，需从模型结果中提取（见下文）
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = 0   # 默认类别 ID，可根据需要修改
            hypothesis.score = 1.0  # 默认置信度
            detection.results.append(hypothesis)

            # 填充边界框（像素坐标）
            detection.bbox.center.x = float(x + w / 2.0)
            detection.bbox.center.y = float(y + h / 2.0)
            detection.bbox.size_x = float(w)
            detection.bbox.size_y = float(h)

            bbox_array.detections.append(detection)

        self.bbox_pub.publish(bbox_array)


    def get_result_detected(self):
        return self.target_detected

    def get_result_info(self):
        return self.target_info

    def run(self):
        rospy.loginfo("YOLO检测节点开始运行")
        rospy.spin()


if __name__ == '__main__':
    try:
        node = YOLODetector()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("节点启动失败: %s", e)