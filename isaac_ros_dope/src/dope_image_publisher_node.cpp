#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rcl_yaml_param_parser/parser.h"
#if __GNUC__ < 9
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif

using namespace std::chrono_literals;

class DopeImagePublisher : public rclcpp::Node
{
  public:
    DopeImagePublisher(rclcpp::NodeOptions options)
    : Node("dope_image_publisher", options)
    , config_filename_(declare_parameter<std::string>("configuration_file", "dope_config.yaml"))
    , image_filename_(declare_parameter<std::string>("image_file", "img.jpg"))
    {
      
      pub_ = image_transport::create_camera_publisher(this, "image");
      
      /// Open configuration YAML file
      const std::string package_directory = ament_index_cpp::get_package_share_directory("isaac_ros_dope");
      fs::path yaml_path = package_directory / fs::path("config") / fs::path(config_filename_);
      if (!fs::exists(yaml_path)) 
      {
        RCLCPP_ERROR(this->get_logger(), "%s could not be found. Exiting.", yaml_path.string().c_str());
      }
      else
      {
        // Parse the YAML file and get the camera matrix
        rcl_params_t * dope_params = rcl_yaml_node_struct_init(rcutils_get_default_allocator());
        rcl_parse_yaml_file(yaml_path.c_str(), dope_params);
        rcl_variant_t * cam_mat = rcl_yaml_node_struct_get("dope", "camera_matrix", dope_params);
        if (cam_mat->double_array_value != nullptr) 
        {
          auto vv = cam_mat->double_array_value->values;
          camera_matrix_ = {vv[0], vv[1], vv[2], vv[3], vv[4], vv[5], vv[6], vv[7], vv[8]};
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "No camera_matrix parameter found");
        }
      }
      /// Image
      RCLCPP_INFO(this->get_logger(), "File name for publishing image is : %s", image_filename_.c_str());
      image_ = cv::imread(image_filename_, cv::IMREAD_COLOR);
      if (image_.empty()) 
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to load image (%s):",image_filename_.c_str()); 
      }

      /// Create the timer
      timer_ = this->create_wall_timer(500ms, std::bind(&DopeImagePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      //RCLCPP_INFO(this->get_logger(), "Publishing image: ");
      //RCLCPP_INFO(this->get_logger(), "%f",camera_matrix_[0]);
      
      sensor_msgs::msg::Image::SharedPtr out_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
      out_img->header.frame_id = "camera";
      out_img->header.stamp = rclcpp::Clock().now();
      camera_info_.header.frame_id = "camera";
      camera_info_.header.stamp = rclcpp::Clock().now();//this->get_clock()->now();
      camera_info_.width = image_.cols;
      camera_info_.height = image_.rows;
      camera_info_.distortion_model = "plumb_bob";
      camera_info_.d = {0, 0, 0, 0, 0};
      camera_info_.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
      /// Intrinsic camera matrix for the raw (distorted) images.
      ///     [fx  0 cx]
      /// K = [ 0 fy cy]
      ///     [ 0  0  1]
      camera_info_.k = {camera_matrix_[0], camera_matrix_[1], camera_matrix_[2],
                        camera_matrix_[3], camera_matrix_[4], camera_matrix_[5],
                        camera_matrix_[6], camera_matrix_[7], camera_matrix_[8]};

      camera_info_.p = {camera_matrix_[0], camera_matrix_[1], camera_matrix_[2], 0,
                        camera_matrix_[3], camera_matrix_[4], camera_matrix_[5], 0,
                        camera_matrix_[6], camera_matrix_[7], camera_matrix_[8], 0};

      pub_.publish(*out_img, camera_info_);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::CameraPublisher pub_;
    sensor_msgs::msg::CameraInfo camera_info_;
    const std::string config_filename_;
    const std::string image_filename_;
    std::array<double, 9> camera_matrix_ = {0};
    cv::Mat image_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<DopeImagePublisher>(options));

  rclcpp::shutdown();
  return 0;
}
