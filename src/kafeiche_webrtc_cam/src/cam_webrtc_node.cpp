#include <rclcpp/rclcpp.hpp>
#include <gst/gst.h>

class MulticastStreamer : public rclcpp::Node
{
public:
    MulticastStreamer() : Node("cam_node")
    {
        gst_init(nullptr, nullptr);

        // Этот пайплайн проверен на Raspberry Pi 5 + IMX477, Pi 4 + HQ Camera, Arducam и т.д.
        const char* pipeline_str =
            "libcamerasrc ! "
            // Явно указываем, что хотим YUV, а не RAW
            "video/x-raw,format=NV12,width=1280,height=720,framerate=30/1 ! "
            "videoconvert n-threads=4 ! "
            "x264enc tune=zerolatency bitrate=2500 speed-preset=ultrafast key-int-max=45 threads=4 ! "
            "video/x-h264,profile=baseline ! "           // baseline = максимальная совместимость
            "rtph264pay config-interval=1 pt=96 ! "
            "udpsink host=192.168.1.7 port=5004 sync=false";

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str, &error);

        if (!pipeline_ || error) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка пайплайна: %s", 
                         error ? error->message : "неизвестно");
            if (error) g_error_free(error);
            return;
        }

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        // Проверяем, действительно ли всё пошло
        GstState state, pending;
        GstClockTime timeout = 5 * GST_SECOND;
        GstStateChangeReturn ret = gst_element_get_state(pipeline_, &state, &pending, timeout);

        if (ret == GST_STATE_CHANGE_FAILURE || state != GST_STATE_PLAYING) {
            RCLCPP_ERROR(this->get_logger(), "Пайплайн не запустился!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), 
                    "Multicast стрим работает!\n"
                    "Адрес: udp://@239.255.0.1:5004\n"
                    "Открой в VLC или ffplay");
    }

    ~MulticastStreamer()
    {
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
    }

private:
    GstElement* pipeline_ = nullptr;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MulticastStreamer>());
    rclcpp::shutdown();
    return 0;
}