#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher() : Node("image_publisher"), 
                       rgb_image_path_("/home/ubuntuhjx/openarm_ws/src/openarm_cam_pkg/rgb_image.png"),
                       depth_image_path_("/home/ubuntuhjx/openarm_ws/src/openarm_cam_pkg/depth_image.png")
    {
        // 加载RGB图像
        rgb_image_ = cv::imread(rgb_image_path_);
        if (rgb_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load RGB image from: %s", rgb_image_path_.c_str());
            RCLCPP_INFO(this->get_logger(), "Will create a default test image instead.");
            rgb_image_ = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));
            cv::putText(rgb_image_, "RGB Test Image", cv::Point(200, 240), 
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully loaded RGB image from: %s", rgb_image_path_.c_str());
        }

        // 加载深度图像 - 注意：深度图像通常是单通道
        // 使用 IMREAD_UNCHANGED 保持原始格式
        depth_image_ = cv::imread(depth_image_path_, cv::IMREAD_UNCHANGED);
        
        if (depth_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load depth image from: %s", depth_image_path_.c_str());
            RCLCPP_INFO(this->get_logger(), "Will create a default test depth image instead.");
            
            // 创建深度图像（16位单通道）
            depth_image_ = cv::Mat(480, 640, CV_16UC1, cv::Scalar(1000));
            cv::rectangle(depth_image_, cv::Rect(200, 150, 200, 200), cv::Scalar(3000), -1);
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully loaded depth image from: %s", depth_image_path_.c_str());
            
            // 如果深度图像是三通道，转换为单通道
            if (depth_image_.channels() == 3) {
                cv::cvtColor(depth_image_, depth_image_, cv::COLOR_BGR2GRAY);
                // 转换为16位
                depth_image_.convertTo(depth_image_, CV_16UC1, 256.0);
            }
        }
        
        rgb_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("rgb_image", 10);
        depth_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("depth_image", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30), 
                                         std::bind(&ImagePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto now = this->now();
        static int rgb_count = 0;
        static int depth_count = 0;
        
        // 发布RGB图像
        try {
            auto rgb_msg = std::make_unique<sensor_msgs::msg::Image>();
            auto rgb_cv_image = std::make_unique<cv_bridge::CvImage>();
            
            rgb_cv_image->encoding = "bgr8";
            rgb_cv_image->image = rgb_image_.clone();
            rgb_cv_image->header.stamp = now;
            rgb_cv_image->header.frame_id = "camera_frame";
            
            *rgb_msg = *rgb_cv_image->toImageMsg();
            rgb_publisher_->publish(std::move(rgb_msg));
            
            if (rgb_count++ % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), "Publishing RGB image (size: %dx%d)", 
                           rgb_image_.cols, rgb_image_.rows);
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "RGB cv_bridge exception: %s", e.what());
        }
        
        // 发布深度图像
        try {
            auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
            auto depth_cv_image = std::make_unique<cv_bridge::CvImage>();
            
            // 根据深度图像类型设置编码
            if (depth_image_.type() == CV_16UC1) {
                depth_cv_image->encoding = "16UC1";
            } else if (depth_image_.type() == CV_32FC1) {
                depth_cv_image->encoding = "32FC1";
            } else {
                // 默认为mono16
                depth_cv_image->encoding = "mono16";
            }
            
            depth_cv_image->image = depth_image_.clone();
            depth_cv_image->header.stamp = now;
            depth_cv_image->header.frame_id = "camera_frame";
            
            *depth_msg = *depth_cv_image->toImageMsg();
            depth_publisher_->publish(std::move(depth_msg));
            
            if (depth_count++ % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), "Publishing depth image (size: %dx%d, type: %d)", 
                           depth_image_.cols, depth_image_.rows, depth_image_.type());
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Depth cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat rgb_image_;
    cv::Mat depth_image_;
    std::string rgb_image_path_;
    std::string depth_image_path_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}