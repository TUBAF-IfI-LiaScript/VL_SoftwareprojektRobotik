// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

cv::Mat imgGrayscale;  
cv::Mat imgBlurred;   
cv::Mat imgCanny;

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    cv::cvtColor(cv_bridge::toCvShare(msg, "bgr8")->image, imgGrayscale, CV_BGR2GRAY); 
    
    cv::GaussianBlur(imgGrayscale,   // input image
        imgBlurred,                  // output image
        cv::Size(5, 5),              // smoothing window width and height in pixels
        1.5);                        // sigma value, determines how much the image will be blurred

    cv::Canny(imgBlurred,            // input image
        imgCanny,                    // output image
        100,                         // low threshold
        200);                        // high threshold   
    
    cv::imshow("view", imgCanny);
    cv::waitKey(10);
  } catch (const cv_bridge::Exception & e) {
    auto logger = rclcpp::get_logger("my_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener", options);
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  rclcpp::spin(node);
  cv::destroyWindow("view");

  return 0;
}
