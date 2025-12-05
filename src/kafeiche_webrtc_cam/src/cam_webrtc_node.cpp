#include <rclcpp/rclcpp.hpp>
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>

class WebRTCStreamer : public rclcpp::Node
{
public:
    WebRTCStreamer() : Node("cam_webrtc_node")
    {
        gst_init(nullptr, nullptr);

        pipeline_ = gst_parse_launch(
            "libcamerasrc ! "
            "video/x-raw,width=1920,height=1080,framerate=20/1 ! "
            "videoconvert ! "
            "x264enc tune=zerolatency bitrate=6000 speed-preset=superfast ! "
            "rtph264pay config-interval=1 pt=96 ! "
            "udpsink host=127.0.0.1 port=5004",
            nullptr);
        

        webrtcbin_ = gst_bin_get_by_name(GST_BIN(pipeline_), "webrtcbin");

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        RCLCPP_INFO(this->get_logger(), "WebRTC camera streamer started");
    }

    ~WebRTCStreamer()
    {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(webrtcbin_);
        gst_object_unref(pipeline_);
    }

private:
    GstElement *pipeline_;
    GstElement *webrtcbin_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebRTCStreamer>());
    rclcpp::shutdown();
    return 0;
}
