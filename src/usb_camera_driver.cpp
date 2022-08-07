/*
Copyright (c) 2019 Andreas Klintberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <experimental/filesystem>

#include "usb_camera_driver.hpp"

using namespace std::chrono_literals;
namespace usb_camera_driver
{

CameraDriver::CameraDriver(const rclcpp::NodeOptions &node_options) : Node("usb_camera_driver", node_options),
idx_write_(0),
idx_read_(0),
frames_counter_in_(0),
frames_counter_out_(0),
frames_counter_out_total(0),
is_running_(true)
{

    frame_id_ = this->declare_parameter("frame_id", "camera");

    image_width_ = this->declare_parameter("image_width", 1280);
    image_height_ = this->declare_parameter("image_height", 720);
    fps_ = this->declare_parameter("fps", 10.0);
    std::cout << "publish images of " << image_width_ << "x" << image_height_ << " at " << fps_ << "(Hz)" << std::endl;

    camera_id = this->declare_parameter("camera_id", 0);

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    camera_info_pub_ = image_transport::create_camera_publisher(this, "image", custom_qos_profile);

    cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);

    /* get ROS2 config parameter for camera calibration file */
    auto camera_calibration_file_param_ = this->declare_parameter("camera_calibration_file", "file://config/camera.yaml");
    cinfo_manager_->loadCameraInfo(camera_calibration_file_param_);

    cap.open(camera_id);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);

    last_frame_ = std::chrono::steady_clock::now();

    save_path_ = this->declare_parameter("save_path", "");
    size_t buffer_size = this->declare_parameter("buffer_size", 100);
    frames_ = std::vector<ImageTime>(buffer_size);

    if (!save_path_.empty())
    {
        std::stringstream save_path_str;
        save_path_str << save_path_ << "/" << "record_" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        save_path_ = save_path_str.str();
        std::cout << "Saving images to " << save_path_ << std::endl;
        save_thread_ = std::thread([this](){SaveToDisk();});
    }
    else
    {
        std::cout << "save_path is empty. Not Saving images." << std::endl;
    }
    last_debug_print_ = std::chrono::steady_clock::now();
    timer_ = this->create_wall_timer(1ms, std::bind(&CameraDriver::ImageCallback, this));
}

CameraDriver::~CameraDriver()
{
    is_running_ = false;
    cv_wait_for_frame_.notify_one();
    save_thread_.join();
    std::cout << "Wrote " << frames_counter_out_total << " frames to " << save_path_ << std::endl;

}

void CameraDriver::SaveToDisk()
{
    if (!std::experimental::filesystem::exists(save_path_))
    {
        std::experimental::filesystem::create_directories(save_path_);
    }
    while (is_running_)
    {
        ImageTime crnt_image;
        std::unique_lock<std::mutex> lock(cv_wait_for_frame_mu_);
        cv_wait_for_frame_.wait(lock, [&]{return (idx_write_ != idx_read_ || !is_running_);} );
        {
            while (idx_write_ != idx_read_)
            {
                // save image on idx_read_
                {
                    std::lock_guard<std::mutex> lock_guard(buffer_mu_);
                    crnt_image = frames_[idx_read_];
                    frames_counter_out_++;
                }
                frames_counter_out_total++;
                crnt_image.Save(save_path_);
                idx_read_ = (idx_read_ + 1) % frames_.size();
            }
        }
    }
}

std::shared_ptr<sensor_msgs::msg::Image> CameraDriver::ConvertFrameToMessage(cv::Mat &frame)
{
    std_msgs::msg::Header header_;
    sensor_msgs::msg::Image ros_image;

    // Make sure output in the size the user wants even if it is not native
    if(frame.rows != image_width_ || frame.cols != image_height_){
        cv::resize(frame, frame, cv::Size(image_width_, image_height_));
    }

    /* To remove CV-bridge and boost-python3 dependencies, this is pretty much a copy of the toImageMsg method in cv_bridge. */
    ros_image.header = header_;
    ros_image.height = frame.rows;
    ros_image.width = frame.cols;
    ros_image.encoding = "bgr8";
    /* FIXME c++20 has std::endian */
    // ros_image.is_bigendian = (std::endian::native == std::endian::big);
    ros_image.is_bigendian = false;
    ros_image.step = frame.cols * frame.elemSize();
    size_t size = ros_image.step * frame.rows;
    ros_image.data.resize(size);

    if (frame.isContinuous())
    {
        memcpy(reinterpret_cast<char *>(&ros_image.data[0]), frame.data, size);
    }
    else
    {
        // Copy by row by row
        uchar *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
        uchar *cv_data_ptr = frame.data;
        for (int i = 0; i < frame.rows; ++i)
        {
            memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
            ros_data_ptr += ros_image.step;
            cv_data_ptr += frame.step;
        }
    }

    auto msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>(ros_image);
    return msg_ptr_;
}

void CameraDriver::ImageCallback()
{
    cap >> frame;

    auto now = std::chrono::steady_clock::now();

    if (!frame.empty() &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_).count() > 1/fps_*1000)
    {
        last_frame_ = now;
        frames_counter_in_++;
        int diff_milli(std::chrono::duration_cast<std::chrono::milliseconds>(last_frame_ - last_debug_print_).count());
        if ( diff_milli > 1000)
        {
            std::cout << "Got  frames at " << frames_counter_in_ / (diff_milli*1e-3) << " (frame/sec)" << std::endl;
            std::cout << "Save frames at " << frames_counter_out_ / (diff_milli*1e-3) << " (frame/sec)" << std::endl;
            frames_counter_in_ = 0;
            last_debug_print_ = last_frame_;
        }

        {
            std::lock_guard<std::mutex> lock_guard(buffer_mu_);
            if (frames_counter_in_ == 0) frames_counter_out_ = 0;
            frames_[idx_write_] = ImageTime(frame, last_frame_);
            cv_wait_for_frame_.notify_one();
        }
        idx_write_ = (idx_write_ + 1) % frames_.size();

        // Convert to a ROS2 image
        if (!is_flipped)
        {
            image_msg_ = ConvertFrameToMessage(frame);
        }
        else
        {
            // Flip the frame if needed
            cv::flip(frame, flipped_frame, 1);
            image_msg_ = ConvertFrameToMessage(frame);
        }

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_(
            new sensor_msgs::msg::CameraInfo(cinfo_manager_->getCameraInfo()));

        rclcpp::Time timestamp = this->get_clock()->now();

        image_msg_->header.stamp = timestamp;
        image_msg_->header.frame_id = frame_id_;

        camera_info_msg_->header.stamp = timestamp;
        camera_info_msg_->header.frame_id = frame_id_;

        camera_info_pub_.publish(image_msg_, camera_info_msg_);
    }
}

bool ImageTime::Save(std::string save_path)
{
    std::stringstream filename;
    filename << save_path << "/image_" << std::chrono::duration_cast<std::chrono::milliseconds>(timestamp_.time_since_epoch()).count() << ".jpeg";
    cv::imwrite(filename.str(), image_);
    // std::cout << "Wrote " << filename.str() << std::endl;
    return true;
}
} // namespace usb_camera_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(usb_camera_driver::CameraDriver)
