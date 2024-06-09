#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class LineFollower : public rclcpp::Node {
public:
  LineFollower() : Node("line_follower") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, [this](sensor_msgs::msg::Image::SharedPtr msg) {
        this->listenerCallback(msg);
      });

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    linear_velocity_gain_ = 0.2;
    angular_velocity_gain_ = 0.3;
  }

private:
  void listenerCallback(sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Receiving video frame");

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img = cv_ptr->image;
    cv::resize(img, img, cv::Size(), 0.5, 0.5, cv::INTER_CUBIC);

    cv::Mat imgHSV;
    cv::cvtColor(img, imgHSV, cv::COLOR_BGR2HSV);

    cv::Scalar lower_yellow(0, 35, 0);
    cv::Scalar upper_yellow(57, 255, 255);

    cv::Mat mask;
    cv::inRange(imgHSV, lower_yellow, upper_yellow, mask);

    int roi_height = mask.rows / 4;
    cv::Mat roi = mask(cv::Range(mask.rows - roi_height, mask.rows), cv::Range::all());

    cv::Moments M = cv::moments(roi);
    if (M.m00 > 0) {
      int cx = M.m10 / M.m00;
      int cy = M.m01 / M.m00 + mask.rows - roi_height;
      cv::Point centroid(cx, cy);

      int radius = 8;
      cv::Point img_center(mask.cols / 2, mask.rows / 2);
      double distance_from_center = std::sqrt(std::pow(cx - img_center.x, 2) + std::pow(cy - img_center.y, 2));
      double max_distance = std::sqrt(std::pow(img_center.x, 2) + std::pow(img_center.y, 2));
      int color_intensity = static_cast<int>(255 * (1 - distance_from_center / max_distance));
      cv::Scalar color(color_intensity, color_intensity, 0);
      cv::circle(img, centroid, radius, color, -1);

      // Calculate linear and angular velocities based on centroid position
      int error = img_center.x - cx;
      double linear_velocity = linear_velocity_gain_;
      double angular_velocity = angular_velocity_gain_ * error;

      // Keep the centroid in the center of the ROI
      if (std::abs(error) > 20) {
        angular_velocity = std::clamp(angular_velocity, -0.3, 0.3);
      }

      // Publish the velocity commands
      geometry_msgs::msg::Twist twist;
      twist.linear.x = linear_velocity;
      twist.angular.z = angular_velocity;
      publisher_->publish(twist);
    }

    cv::imshow("Line Threshold", mask);
    cv::imshow("Region of Interest", roi);
    cv::imshow("Centroid Indicator", img);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  double linear_velocity_gain_;
  double angular_velocity_gain_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LineFollower>();
  rclcpp::spin(node);
  // node->destroy_node();
  rclcpp::shutdown();
  return 0;
}