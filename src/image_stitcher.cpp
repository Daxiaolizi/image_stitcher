//#include "image_stitcher.h"
#include "../include/image_stitcher/image_stitcher.h"
#include <ros/package.h>
#include <vector>

ImageStitcher::ImageStitcher(ros::NodeHandle &nh)
    : nh_(nh), it_(nh), calibrating_(false), left_clicked_(false), right_clicked_(false)
 {
   nh_.param<std::string>("left_topic", left_topic_, "/hk_camera_left/image_raw");
   nh_.param<std::string>("right_topic", right_topic_, "/hk_camera_right/image_raw");
   nh_.param<std::string>("output_topic", output_topic_, "/hk_stitched_image");
   nh_.param<std::string>("left_window_name", left_window_name_, "Left Image");
   nh_.param<std::string>("right_window_name", right_window_name_, "Right Image");
   nh_.param<std::string>("stitched_window_name", stitched_window_name_, "Stitched Image");
   nh_.param<double>("display_scale", display_scale_, 0.5);
   nh_.param<int>("circle_radius", circle_radius_, 5);
   nh_.param<double>("font_scale", font_scale_, 0.5);
   nh_.param<int>("font_thickness", font_thickness_, 2);
   nh_.param<int>("wait_key_delay", wait_key_delay_, 30);

   //color
   std::vector<int> circle_color_vec;
   if(nh_.getParam("circle_color", circle_color_vec) && circle_color_vec.size() == 3 ){
       circle_color_ = cv::Scalar(circle_color_vec[0], circle_color_vec[1], circle_color_vec[2]);
   }
   else {
       circle_color_ = cv::Scalar(0, 0, 255);//red
       ROS_WARN("Failed to load circle_color, using default [0, 0, 255]");
   }

   sub_left_ = it_.subscribe(left_topic_, 1, &ImageStitcher::leftImageCallback, this);
     if (sub_left_.getNumPublishers() == 0) {
         ROS_ERROR("No publishers on left_topic %s", left_topic_.c_str());
     }
   sub_right_ = it_.subscribe(right_topic_, 1, &ImageStitcher::rightImageCallback, this);
     if (sub_right_.getNumPublishers() == 0) {
         ROS_ERROR("No publishers on right_topic %s", right_topic_.c_str());
     }
   pub_stitched_ = nh_.advertise<sensor_msgs::Image>(output_topic_, 1);

     cv::namedWindow(left_window_name_, cv::WINDOW_NORMAL);
     cv::namedWindow(right_window_name_, cv::WINDOW_NORMAL);

     cv::setMouseCallback(left_window_name_, ImageStitcher::onMouseLeft, this);
     cv::setMouseCallback(right_window_name_, ImageStitcher::onMouseRight, this);

}

void ImageStitcher::leftImageCallback(const sensor_msgs::ImageConstPtr &msg) {
//    ROS_INFO("收到左图像: 大小 %dx%d, 编码 %s", msg->width, msg->height, msg->encoding.c_str());
    if (!calibrating_) {
        try {
            left_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image.clone();

//            ros::Time now = ros::Time::now();
//            ros::Duration lag = now - msg->header.stamp;
//            ROS_INFO("[左图像] 延迟: %.3f 秒", lag.toSec());
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("左相机回调 cv_bridge 异常: %s", e.what());
        }
    }
     image_updated_ = true;
}

void ImageStitcher::rightImageCallback(const sensor_msgs::ImageConstPtr& msg) {
//    ROS_INFO("收到右图像: 大小 %dx%d, 编码 %s", msg->width, msg->height, msg->encoding.c_str());
    if (!calibrating_) {
        try {
            right_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image.clone();

//            ros::Time now = ros::Time::now();
//            ros::Duration lag = now - msg->header.stamp;
//            ROS_INFO("[右图像] 延迟: %.3f 秒", lag.toSec());
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("右相机回调 cv_bridge 异常: %s", e.what());
        }
    }
     image_updated_ = true;
}


void ImageStitcher::onMouseLeft(int event, int x, int y, int flags, void* userdata) {
    ImageStitcher* self = static_cast<ImageStitcher*>(userdata);
    if (self) self->handleMouseLeft(event, x, y, flags);
}


void ImageStitcher::handleMouseLeft(int event, int x, int y, int flags) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        left_point_ = cv::Point(x / display_scale_, y / display_scale_);
        left_clicked_ = true;
        ROS_INFO("左图像点击: (%d, %d)", left_point_.x, left_point_.y);
    }
}


void ImageStitcher::onMouseRight(int event, int x, int y, int flags, void* userdata) {
    ImageStitcher* self = static_cast<ImageStitcher*>(userdata);
    if (self) self->handleMouseRight(event, x, y, flags);
}

void ImageStitcher::handleMouseRight(int event, int x, int y, int flags) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        right_point_ = cv::Point(x / display_scale_, y / display_scale_);
        right_clicked_ = true;
        ROS_INFO("右图像点击: (%d, %d)", right_point_.x, right_point_.y);
    }
}


void ImageStitcher::displayImages() {
    if (!left_image_.empty()) {
        cv::Mat display_left;
        cv::resize(left_image_, display_left, cv::Size(), display_scale_, display_scale_);
        if (left_clicked_) {
            cv::Point scaled_point(left_point_.x * display_scale_, left_point_.y * display_scale_);
            cv::circle(display_left, scaled_point, circle_radius_, circle_color_, -1);
            std::string text = "(" + std::to_string(left_point_.x) + ", " + std::to_string(left_point_.y) + ")";
            cv::putText(display_left, text, scaled_point + cv::Point(10, 10),
                        cv::FONT_HERSHEY_SIMPLEX, font_scale_, text_color_, font_thickness_);
        }
        cv::imshow(left_window_name_, display_left);
    } else {
        ROS_WARN("Left image is empty, skipping display.");
    }

    if (!right_image_.empty()) {
        cv::Mat display_right;
        cv::resize(right_image_, display_right, cv::Size(), display_scale_, display_scale_);
        if (right_clicked_) {
            cv::Point scaled_point(right_point_.x * display_scale_, right_point_.y * display_scale_);
            cv::circle(display_right, scaled_point, circle_radius_, circle_color_, -1);
            std::string text = "(" + std::to_string(right_point_.x) + ", " + std::to_string(right_point_.y) + ")";
            cv::putText(display_right, text, scaled_point + cv::Point(10, 10),
                        cv::FONT_HERSHEY_SIMPLEX, font_scale_, text_color_, font_thickness_);
        }
        cv::imshow(right_window_name_, display_right);
    } else {
        ROS_WARN("Right image is empty, skipping display.");
    }
}

void ImageStitcher::stitchImages() {
    ros::Time t1 = ros::Time::now();
    if (left_image_.empty() || right_image_.empty()) {
        ROS_WARN("图像为空，跳过拼接");
        return;
    }

    // 如果用户点击了左右图像，进行校准
    if (left_clicked_ && right_clicked_) {
        int dx = left_point_.x - right_point_.x;
        int dy = left_point_.y - right_point_.y;
        offset_ = cv::Point(dx, dy);
        has_calibrated_ = true;

        ROS_INFO("已完成手动校准，dx = %d, dy = %d", dx, dy);
        left_clicked_ = false;
        right_clicked_ = false;
        calibrating_ = false;
    }

    if (!has_calibrated_) {
        // 尚未完成点击校准，不做拼接
        return;
    }

    int dx = offset_.x;
    int dy = offset_.y;

    // 以下拼接逻辑不变
    int W1 = left_image_.cols;
    int H1 = left_image_.rows;
    int W2 = right_image_.cols;
    int H2 = right_image_.rows;

    int left = std::min(0, dx);
    int top = std::min(0, dy);
    int right = std::max(W1, dx + W2);
    int bottom = std::max(H1, dy + H2);
    int W = right - left;
    int H = bottom - top;
    cv::Mat stitched = cv::Mat::zeros(H, W, left_image_.type());

    cv::Rect roi1(-left, -top, W1, H1);
    cv::Rect intersect1 = roi1 & cv::Rect(0, 0, W, H);
    if (!intersect1.empty()) {
        cv::Mat src_roi1 = left_image_(cv::Rect(intersect1.x + left, intersect1.y + top,
                                                intersect1.width, intersect1.height));
        src_roi1.copyTo(stitched(intersect1));
    }

    cv::Rect roi2(dx - left, dy - top, W2, H2);
    cv::Rect intersect2 = roi2 & cv::Rect(0, 0, W, H);
    if (!intersect2.empty()) {
        cv::Mat src_roi2 = right_image_(cv::Rect(intersect2.x - (dx - left), intersect2.y - (dy - top),
                                                 intersect2.width, intersect2.height));
        src_roi2.copyTo(stitched(intersect2));
    }

    cv::Mat display_stitched;
    cv::resize(stitched, display_stitched, cv::Size(), display_scale_, display_scale_);
    cv::imshow(stitched_window_name_, display_stitched);

    // 发布拼接图像
    cv_bridge::CvImage cv_img;
    cv_img.encoding = "bgr8";
    cv_img.image = stitched;
    sensor_msgs::ImagePtr msg = cv_img.toImageMsg();
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "stitched_frame";
    pub_stitched_.publish(msg);
    ros::Time t2 = ros::Time::now();
    ROS_INFO("拼接耗时: %.3f 秒", (t2 - t1).toSec());

    ROS_INFO("拼接图像已发布至 %s", output_topic_.c_str());
}

void  ImageStitcher::run() {
    // 等待图像准备好
    ROS_INFO("等待图像到达...");
    ros::Rate wait_rate(10);
    while (ros::ok() && (left_image_.empty() || right_image_.empty())) {
        wait_rate.sleep();
    }
    ROS_INFO("图像已准备好，开始运行。");
//    ros::Duration(1.0).sleep();  // 等待 1s 让图像到来
    ros::Rate rate(60);
    while (ros::ok()) {
        displayImages();

        char key = cv::waitKey(wait_key_delay_);
        if (key == '1') {
            calibrating_ = true;
            left_clicked_ = false;
            right_clicked_ = false;
            has_calibrated_ = false;  // 清除旧 offset
            ROS_INFO("校准已开始。请在图像上点击选择点。");
        } else if (key == '2') {
            if (left_clicked_) {
                left_clicked_ = false;
                ROS_INFO("1: 已撤销左图像上的最后一次点击");
            } else if (right_clicked_) {
                right_clicked_ = false;
                ROS_INFO("1: 已撤销右图像上的最后一次点击");
            }
        }
        else if (key == 'q')
        {
            break;
        }
            stitchImages();
            image_updated_ = false;
        }

        displayImages();  // 不影响回调
        rate.sleep();

//        cv::destroyAllWindows();
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "image_stitcher");
    ros::NodeHandle nh;
    ImageStitcher stitcher(nh);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    stitcher.run();
    ros::waitForShutdown();
    return 0;

}