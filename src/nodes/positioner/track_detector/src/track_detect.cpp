#include <chrono>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
//#include <std_msgs/msg/string.hpp>  // subscribing to a string topic_
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/cuda.hpp>
using namespace cv::cuda;

using namespace std::chrono_literals;


class VideoTrackDetector : public rclcpp::Node {
public:
    VideoTrackDetector() : Node("track_detect_main") {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "received_image", 10, std::bind(&VideoTrackDetector::processImageCallback, this, std::placeholders::_1));

        processed_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("track_line", 10); //this->create_..
     // Timer to control publishing rate

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&VideoTrackDetector::publishProcessedImage, this)); //this->create_.. 
    }

  

private:
    void processImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Process the received message (e.g., display it)
        RCLCPP_INFO(this->get_logger(), "Received image message");
        
        try{
           // Convert ROS image message to OpenCV format
           cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

           //process the image
           cv::Mat processed_image;
           processPipeline(cv_ptr->image, processed_image);
     
           video_monitoring_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", processed_image).toImageMsg(); //instead of "mono8"(testrun gray scale) set to: cv_ptr->encoding

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
            return;
        }
    }

    void processPipeline(const cv::Mat& input_image, cv::Mat& output_image){
        //Processing pipeline implementation

        //1. Cut relevant field of view --> Cut Horizontal
        int roi_y = 460;          //Starting y-coordinate (horizon in image)
        int roi_height = input_image.rows - roi_y;   //height of the upper part to be cut

        cv::Rect roi_rect(0, roi_y, input_image.cols, roi_height);
        cv::Mat cropped_image = input_image(roi_rect).clone();

        //2. Convert to birts eye view
                //source points
        std::vector<cv::Point2f> src_points ={
            cv::Point2f(650, 0),                  //top-left corner
            cv::Point2f(750, 0),                     //top-right corner
            cv::Point2f(200, cropped_image.rows),                     //bottom-left corner
            cv::Point2f(1200, cropped_image.rows)   //bottom-right corner
        };
                //destination points
        std::vector<cv::Point2f> dst_points = {
            cv::Point2f(0,0),                                       //top-left corner
            cv::Point2f(cropped_image.cols, 0),                     //top-right corner
            cv::Point2f(0, cropped_image.rows),                    //bottom-left corner
            cv::Point2f(cropped_image.cols, cropped_image.rows)   //bottom-right corner
        };
                //perspective transformation matrix
            cv::Mat perspective_matrix = cv::getPerspectiveTransform(src_points, dst_points);

                //Perspective Transformation
            cv::Mat birds_eye_view;
            cv::warpPerspective(cropped_image, birds_eye_view, perspective_matrix, cropped_image.size());

        //3. Cut roi
       //4. Convert to GrayScale
            cv::Mat gray_image;
            cv::cvtColor(birds_eye_view, gray_image, cv::COLOR_BGR2GRAY); // COLOR_BGR2GRAY does not work, why? 

        //5. Apply histogram equalization
            cv::Mat equalized_image;
            cv::equalizeHist(gray_image, equalized_image);

        //6. Apply Gaussian Blur
            cv::Mat blurred_image;
            cv::GaussianBlur(equalized_image, blurred_image, cv::Size(5,5),0);

        //7. Apply edge detection (e.g. Canny)
            
        cv::Mat edges;
        cv::Canny(blurred_image, edges, 50, 150);
      

        //8. Sliding Windows

        //9. Bezier curve fitting

        //10. Dynamic Mask

        //11. Put together edges and original image    

         /*
           cv::Mat result;
           blurred_image.copyTo(result);
           result.setTo(cv::Scalar(245,245,0), edges);  */
            output_image = edges;// result;
    }

    void publishProcessedImage (){
        //Check if a processed iage is available
        if (!video_monitoring_) {
                  
            return;
        }
          //publish modified Image
         RCLCPP_INFO(this->get_logger(), "PUBLISHED image message");
         processed_image_publisher_->publish(*video_monitoring_);
        

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_publisher_;
    sensor_msgs::msg::Image::SharedPtr video_monitoring_; 
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoTrackDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

