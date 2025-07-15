#ifndef IMAGE_STITCHER_H
#define IMAGE_STITCHER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class ImageStitcher {
public:
    ImageStitcher(ros::NodeHandle& nh);
    void run();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_left_;
    image_transport::Subscriber sub_right_;
    ros::Publisher pub_stitched_;

    cv::Mat left_image_;
    cv::Mat right_image_;
    cv::Point left_point_;
    cv::Point right_point_;
    bool calibrating_;
    bool left_clicked_;
    bool right_clicked_;
    bool image_updated_;
    bool display_enabled_ = true;



    std::string left_topic_;
    std::string right_topic_;
    std::string output_topic_;
    std::string left_window_name_;
    std::string right_window_name_;
    std::string stitched_window_name_;
    double display_scale_;
    int circle_radius_;
    cv::Point offset_;
    bool has_calibrated_ = false;
    cv::Scalar circle_color_;
    double font_scale_;
    int font_thickness_;
    cv::Scalar text_color_;
    int wait_key_delay_; //参数

//    void loadParameters(const std::string& config_file);
    void leftImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rightImageCallback(const sensor_msgs::ImageConstPtr& msg);
    static void onMouseLeft(int event, int x, int y, int flags, void* userdata);
    static void onMouseRight(int event, int x, int y, int flags, void* userdata);
    void handleMouseLeft(int event, int x, int y, int flags);
    void handleMouseRight(int event, int x, int y, int flags);
    void displayImages();
    void syncImageCallback(const sensor_msgs::ImageConstPtr& left_msg,
                           const sensor_msgs::ImageConstPtr& right_msg);
    void stitchImages();


};

#endif