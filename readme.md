# scan_land包

## 需要关注
### 1. src/fly_mission.cpp
此为示例，请对着这个复制粘贴
0. 引用(conllision... 用于引入control_by_vel函数satfunc函数)
1. 全局变量与宏定义
2. 回调函数
3. sub与pub
4. 参数读取
5. 添加于起飞与悬停过程，用于向起飞检测发布任务信号
6. 包含发布任务检测信号，状态机
7. 向检测节点发布退出信号(还有点问题)，可加可不加

### 2. src/collision_avoidance.h
需要这里的control_by_vel和satfunc(其中control... 需要当前位置回调local_pos)

### 3. config/scan_land.yaml
1. 用于检测节点
- model_path yolo模型路径，用绝对路径
- confidence_threshold 置信度，可筛选yolo结果(越高越严格)
- nms_threshold nms去重的参数，可不动
- input_width yolo处理图像大小，由yolo模型决定
- input_height 同上
2. 用于任务节点
- p_xy 追踪速度系数
- vel_track_max 追踪速度限制
- time_threshold follow_mode持续的最短时间
- yolo_follow_kp follow_mode对齐目标系数

### 4. shell/
