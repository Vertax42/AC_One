#include <librealsense2/rs.hpp>
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

std::vector<std::string> getRSSerialNum()
{
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    std::vector<std::string> list_sn;

    if (devices.size() == 0)
    {
        std::cerr << "No RealSense device detected. Please connect a device.\n";
        return list_sn;
    }

    std::cout << "Found " << devices.size() << " device(s):\n";

    for (rs2::device device: devices)
    {
        if (device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
        {
            std::string sn = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            std::string usb_type = "N/A";

            if (device.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
            {
                usb_type = device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
            }

            list_sn.push_back(sn);

            std::cout << "  - Serial: " << sn << " , USB: " << usb_type << "\n";
        }
    }

    return list_sn;
}

class RealSenseNode : public rclcpp::Node
{
public:
    RealSenseNode() : Node("list_camera_node")
    {
        serials_ = getRSSerialNum();

        if (serials_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No RealSense devices found.");

            return;
        }

        // 启动每个相机的 pipeline
        for (const auto &sn: serials_)
        {
            rs2::config cfg;
            cfg.enable_device(sn);
            cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

            rs2::pipeline pipe;
            pipe.start(cfg);
            pipelines_.push_back(std::move(pipe));
        }

        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]
                { capture(); });
    }

private:
    void capture()
    {
        for (size_t i = 0; i < pipelines_.size(); ++i)
        {
            rs2::frameset frames = pipelines_[i].wait_for_frames();
            rs2::frame color = frames.get_color_frame();

            if (color)
            {
                cv::Mat image(cv::Size(640, 480), CV_8UC3,
                              (void *) color.get_data(), cv::Mat::AUTO_STEP);

                cv::imshow(serials_[i], image);
            }
        }

        std::cout << "Press any key in the image window(s) to exit...\n";
        cv::waitKey(0);

        rclcpp::shutdown();
    }

    std::vector<std::string> serials_;
    std::vector<rs2::pipeline> pipelines_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSenseNode>();
    rclcpp::spin(node);

    return 0;
}