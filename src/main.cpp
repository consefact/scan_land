#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

// 定义颜色范围结构体
struct ColorRange {
    string name;
    Scalar lower;
    Scalar upper;
    Scalar displayColor;
};

// 初始化颜色范围
vector<ColorRange> initColorRanges() {
    return {
        // 红色（两个范围，因为HSV中红色在0°和180°附近）
        {"Red", Scalar(0, 120, 70), Scalar(10, 255, 255), Scalar(0, 0, 255)},
        {"Red", Scalar(170, 120, 70), Scalar(180, 255, 255), Scalar(0, 0, 255)},
        {"Green", Scalar(35, 50, 50), Scalar(85, 255, 255), Scalar(0, 255, 0)},
        {"Blue", Scalar(100, 150, 0), Scalar(140, 255, 255), Scalar(255, 0, 0)},
        {"Yellow", Scalar(20, 100, 100), Scalar(30, 255, 255), Scalar(0, 255, 255)},
        {"Orange", Scalar(10, 100, 100), Scalar(20, 255, 255), Scalar(0, 165, 255)},
        {"Purple", Scalar(140, 50, 50), Scalar(160, 255, 255), Scalar(255, 0, 255)},
        {"Cyan", Scalar(85, 100, 100), Scalar(100, 255, 255), Scalar(255, 255, 0)}
    };
}

// 检测并标记颜色
void detectColors(Mat& frame, Mat& result) {
    // 转换为HSV颜色空间
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    
    // 获取颜色范围
    vector<ColorRange> colors = initColorRanges();
    
    // 创建掩码和输出图像
    Mat mask, colorMask;
    result = frame.clone();
    
    for (const auto& color : colors) {
        // 创建颜色掩码
        inRange(hsv, color.lower, color.upper, mask);
        
        // 形态学操作（去除噪声）
        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(mask, mask, MORPH_OPEN, kernel);
        morphologyEx(mask, mask, MORPH_CLOSE, kernel);
        
        // 查找轮廓
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        
        // 绘制轮廓和标签
        for (size_t i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area > 500) { // 忽略小区域
                // 计算轮廓的边界矩形
                Rect rect = boundingRect(contours[i]);
                
                // 绘制轮廓
                drawContours(result, contours, i, color.displayColor, 2);
                
                // 绘制边界矩形
                rectangle(result, rect, color.displayColor, 2);
                
                // 添加文本标签
                string label = color.name + " (" + to_string((int)area) + ")";
                putText(result, label, Point(rect.x, rect.y - 10),
                        FONT_HERSHEY_SIMPLEX, 0.7, color.displayColor, 2);
            }
        }
    }
}

// 实时摄像头颜色检测
void realTimeColorDetection() {
    VideoCapture cap(0); // 打开默认摄像头
    if (!cap.isOpened()) {
        cout << "无法打开摄像头" << endl;
        return;
    }
    
    namedWindow("原始图像", WINDOW_AUTOSIZE);
    namedWindow("颜色检测", WINDOW_AUTOSIZE);
    
    Mat frame, result;
    
    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        
        // 检测颜色
        detectColors(frame, result);
        
        // 显示结果
        imshow("原始图像", frame);
        imshow("颜色检测", result);
        
        // 按ESC退出
        char key = waitKey(30);
        if (key == 27) break;
    }
    
    cap.release();
    destroyAllWindows();
}

// 图像文件颜色检测
void imageColorDetection(const string& imagePath) {
    Mat image = imread(imagePath);
    if (image.empty()) {
        cout << "无法读取图像: " << imagePath << endl;
        return;
    }
    
    Mat result;
    detectColors(image, result);
    
    // 显示结果
    namedWindow("原始图像", WINDOW_AUTOSIZE);
    namedWindow("颜色检测", WINDOW_AUTOSIZE);
    
    imshow("原始图像", image);
    imshow("颜色检测", result);
    
    waitKey(0);
    destroyAllWindows();
}

// 精确颜色提取（返回特定颜色的掩码）
Mat extractColor(const Mat& image, const string& colorName) {
    Mat hsv;
    cvtColor(image, hsv, COLOR_BGR2HSV);
    Mat mask = Mat::zeros(image.size(), CV_8UC1);
    
    vector<ColorRange> colors = initColorRanges();
    
    for (const auto& color : colors) {
        if (color.name == colorName) {
            Mat tempMask;
            inRange(hsv, color.lower, color.upper, tempMask);
            bitwise_or(mask, tempMask, mask);
        }
    }
    
    return mask;
}

int main() {
    cout << "选择模式：" << endl;
    cout << "1. 摄像头实时检测" << endl;
    cout << "2. 图像文件检测" << endl;
    cout << "3. 提取特定颜色" << endl;
    
    int choice;
    cin >> choice;
    
    switch (choice) {
        case 1:
            realTimeColorDetection();
            break;
        case 2: {
            string imagePath;
            cout << "输入图像路径: ";
            cin >> imagePath;
            imageColorDetection(imagePath);
            break;
        }
        case 3: {
            string imagePath, colorName;
            cout << "输入图像路径: ";
            cin >> imagePath;
            cout << "输入颜色名称(Red/Green/Blue/Yellow/Orange/Purple/Cyan): ";
            cin >> colorName;
            
            Mat image = imread(imagePath);
            if (image.empty()) {
                cout << "无法读取图像" << endl;
                return -1;
            }
            
            Mat mask = extractColor(image, colorName);
            
            // 应用掩码
            Mat result;
            image.copyTo(result, mask);
            
            namedWindow("原始图像", WINDOW_AUTOSIZE);
            namedWindow("颜色掩码", WINDOW_AUTOSIZE);
            namedWindow("提取结果", WINDOW_AUTOSIZE);
            
            imshow("原始图像", image);
            imshow("颜色掩码", mask);
            imshow("提取结果", result);
            
            waitKey(0);
            destroyAllWindows();
            break;
        }
        default:
            cout << "无效选择" << endl;
    }
    
    return 0;
}