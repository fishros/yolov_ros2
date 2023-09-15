import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose, Detection2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import yolov5


class YOLOv5Ros2(Node):
    def __init__(self):
        super().__init__('yolov5_ros2')
        
        # 声明ROS参数，用于配置YOLOv5节点的行为
        self.declare_parameter("device", "cpu", ParameterDescriptor(
            name="device", description="计算设备选择，默认：cpu 其他示例:cuda:0"))
        self.declare_parameter("model_path", "", ParameterDescriptor(
            name="model_path", description="YOLOv5模型路径，默认为空"))
        self.declare_parameter("image_topic", "/image_raw", ParameterDescriptor(
            name="image_topic", description="输入图像话题，默认：/image_raw"))
        self.declare_parameter("pub_result_image", False, ParameterDescriptor(
            name="pub_result_img", description="是否发布识别结果图像，默认：False"))
        
        # 获取ROS参数的值
        self.device = self.get_parameter('device').value
        self.model_path = self.get_parameter('model_path').value
        self.image_topic = self.get_parameter('image_topic').value
        self.pub_result_image = self.get_parameter('pub_result_image').value
        # 加载模型和工具
        if self.model_path == "":
            package_share_directory = get_package_share_directory('yolov5_ros2')
            self.model_path = package_share_directory + "/config/yolov5n.pt"
        self.yolov5_model = yolov5.load(self.model_path, self.device)
        self.bridge = CvBridge()
        # 创建订阅发布者
        self.yolo_result_pub = self.create_publisher(
            Detection2DArray, "yolo_result", 10)
        if self.pub_result_image:
            self.result_img_pub = self.create_publisher(
                Image, "yolo_result_image", 10)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)
        
    def pub_result(self, result, header):
        """
        发布YOLOv5的识别结果
        
        Args:
            result: YOLOv5的识别结果数据
            header: ROS消息头信息
        """
        result_msg = Detection2DArray()
        result_msg.header = header # 同步 header

        predictions = result.pred[0]  # 获取结果张量
        boxes = predictions[:, :4]  # 框的坐标：x1, y1, x2, y2
        scores = predictions[:, 4]  # 置信度
        categories = predictions[:, 5]  # 类别

        for index in range(len(categories)):
            name = result.names[int(categories[index])]  # 根据分类id查询名字
            score = round(scores[index].item(), 2)  # 保留两位小数
            x1, y1, x2, y2 = map(int, boxes[index])  # 使用 map 转换成int型，方便使用

            detection2d = Detection2D()
            detection2d.id = str(index)
            # 将角点转成边界框类型
            detection2d.bbox.center.position.x = (x1+x2)/2.0
            detection2d.bbox.center.position.y = (y1+y2)/2.0
            detection2d.bbox.size_x = float(x2-x1)
            detection2d.bbox.size_y = float(y2-y1)
            # 存储类型名称和置信度
            obj_pose = ObjectHypothesisWithPose()
            obj_pose.hypothesis.class_id = name
            obj_pose.hypothesis.score = score
            detection2d.results.append(obj_pose)
            result_msg.detections.append(detection2d)

        self.yolo_result_pub.publish(result_msg)

    def pub_result_with_image(self, result, image, header):
        """
        发布包含识别结果的图像
        
        Args:
            result: YOLOv5的识别结果数据
            image: 包含识别结果的图像数据
            header: ROS消息头信息
        """
        predictions = result.pred[0]  # 获取结果张量
        boxes = predictions[:, :4]  # 框的坐标：x1, y1, x2, y2
        scores = predictions[:, 4]  # 置信度
        categories = predictions[:, 5]  # 类别

        for index in range(len(categories)):
            name = result.names[int(categories[index])]  # 根据分类id查询名字
            score = round(scores[index].item(), 2)  # 保留两位小数
            x1, y1, x2, y2 = map(int, boxes[index])  # 使用 map 转换成int型，方便使用
            cv2.rectangle(image, (x1, y1), (x2, y2),
                          (0, 255, 0), 2)  # 在原始图像上绘制矩形框
            cv2.putText(image, f"{name}:{score}", (x1, y1),  # 在矩形框上显示类别名称
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 1)
        result_img_msg = self.bridge.cv2_to_imgmsg(image,encoding="rgb8",header=header)
        self.result_img_pub.publish(result_img_msg)

    def image_callback(self, msg):
        """
        处理输入图像的回调函数
        
        Args:
            msg: 输入图像消息
        """
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="rgb8")
        result = self.yolov5_model(image)
        self.get_logger().info(str(result))
        self.pub_result(result, msg.header)
        if self.pub_result_image:
            self.pub_result_with_image(result, image, msg.header)

def main():
    rclpy.init()
    rclpy.spin(YOLOv5Ros2())
    rclpy.shutdown()

if __name__ == "__main__":
    main()



