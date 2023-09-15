# YOLO_ROS2

基于 YOLOv5 的 ROS 2 封装，允许用户使用给定的模型文件和图像话题进行实时物体检测。

## 1. 安装依赖

本工程依赖 yolov5 库和 ROS 2 vision-msgs 消息接口库，依赖安装方法如下：

```bash
sudo apt update
sudo apt install python3-pip ros-$ROS_DISTRO-vision-msgs
pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple yolov5 
```

## 2. 构建和运行

下载代码到您的工作空间中,编译项目并设置环境变量：

```bash
colcon build
source install/setup.bash
```

现在，您可以运行 yolov5_ros2 节点。通过 -p 参数可以指定检输入话题，比如使用名为`/image`的图像话题。您可以根据需要更改这些参数：

```bash
ros2 run yolov5_ros2 yolov5_ros2 --ros-args -p image_topic:=/image
```

驱动系统相机发布图像可以使用 image_tools 功能包下的 cam2image 节点实现，命令如下：

```
ros2 run image_tools cam2image --ros-args -p device_id:=-1
```

除了该节点，ROS 2 中还可以使用 usb_cam 功能包进行相机驱动，但需要我们手动安装，安装及运行命令如下：

```
sudo apt-get install ros-$ROS_DISTRO-usb-cam
ros2 run usb_cam usb_cam_node_exe
```

## 3. 订阅结果

Yolo_ROS2将检测结果发布到`/yolo_result`话题中。您可以使用以下命令查看检测结果：

```bash
ros2 topic echo /yolo_result
```

如果要查看绘制检测结果图像，请在运行节点时追加参数 `-p pub_result_image:=True` 打开，打开后可以通过 `/yolo_result_image` 话题查看绘制检测结果的图像。


## 4. 更进一步使用

### 4.1 参数设置

在运行Yolo_ROS2节点时，您可以使用 `-p name:=value` 的方式来修改参数值。

#### 4.1.1 图像话题

您可以通过指定以下参数来更改图像话题：

```bash
image_topic:=/image
```

#### 4.1.2 计算设备设置

如果您有CUDA支持的显卡，可以选择以下参数来配置计算设备：

```bash
device:=cpu
```

#### 4.1.3 切换不同Yolov5模型

默认情况下，yolov5_ros2 使用`yolov5n.pt`预训练模型。您可以通过以下参数来更改模型：

```bash
model_path:=/path/to/your/model.pt
```

#### 4.1.4 是否发布结果图像

如果您希望Yolo_ROS2发布检测结果的图像，请使用以下参数：

```bash
pub_result_img:=True
```
运行后可以通过`/yolo_result_image`话题查看检测结果的图像。

