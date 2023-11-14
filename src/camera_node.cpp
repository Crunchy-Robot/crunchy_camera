#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace crunchy {
  class CameraNode : public rclcpp::Node {
  public:
    CameraNode() : Node("camera_node") {
      this->declare_parameter<std::string>("video_device", "/dev/video0");
      this->get_parameter("video_device", video_device_);
    }

    void initialize() {
        image_transport::ImageTransport it(shared_from_this());
        image_publisher_ = it.advertise("image", 10);

        cap_.open(video_device_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera: %s", video_device_.c_str());
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30), std::bind(&CameraNode::timerCallback, this));
    }

  private:
    void timerCallback() {
      cv::Mat frame;
      if (!cap_.read(frame)) {
	RCLCPP_ERROR(this->get_logger(), "Could not read frame from camera");
	return;
      }

      // Convert to ROS image message
      sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      image_publisher_.publish(*msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::Publisher image_publisher_;
    cv::VideoCapture cap_;
    std::string video_device_;
  };
} // namespace crunchy

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<crunchy::CameraNode>();
    // Needs a separate initialization, as shared_from_this() required
    // for ImageTransport can´t safely be used in the constructor.
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
