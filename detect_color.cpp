#include <cv_bridge/cv_bridge.h>
#include "detect_color.h"
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <math.h>

using namespace cv;
using namespace std;

colorDetector::colorDetector()
    : it_(nh_)
{
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &colorDetector::image_cb, this);
    low_h_ = 0;
    high_h_ = 50;
    low_s_ = 5;
    high_s_ = 255;
    low_v_ = 0;
    high_v_ = 255;
    blur_k_size_ = 1;
    dil_k_size_ = 1;
    namedWindow("thresh_img", CV_WINDOW_AUTOSIZE);
    createTrackbar("low_h_", "thresh_img", &low_h_, 179);
    createTrackbar("high_h_", "thresh_img", &high_h_, 179);
    createTrackbar("low_s_", "thresh_img", &low_s_, 255);
    createTrackbar("high_s_", "thresh_img", &high_s_, 255);
    createTrackbar("low_v_", "thresh_img", &low_v_, 255);
    createTrackbar("high_v_", "thresh_img", &high_v_, 255);
    createTrackbar("blur_k_size_", "thresh_img", &blur_k_size_, 15);
    createTrackbar("dil_k_size_", "thresh_img", &dil_k_size_, 15);
}

void colorDetector::image_cb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_gray;
    Mat img_data;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_gray = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cvtColor(cv_gray->image, cv_gray->image, CV_BGR2GRAY);

    vector<Vec3f> circles;
    HoughCircles(cv_gray->image, circles, CV_HOUGH_GRADIENT, 1,(cv_gray->image).rows/6, 50, 25);
    int hello_cnt = 0;

    


    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));
    cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2HSV);

    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int b = cv_ptr->image.at<Vec3b>(cvRound(circles[i][1]), cvRound(circles[i][0]))[0];
        int g = cv_ptr->image.at<Vec3b>(cvRound(circles[i][1]), cvRound(circles[i][0]))[1];
        int r = cv_ptr->image.at<Vec3b>(cvRound(circles[i][1]), cvRound(circles[i][0]))[2];

        int radius = cvRound(circles[i][2]);
        if ( r == 255 and b == 0)
        {
            //printf("B : %d   \n", b);
            //printf("G : %d   \n", g);
            //printf("R : %d   \n", r);
            printf("(x,y) = (%d, %d) \n", cvRound(circles[i][0]), cvRound(circles[i][1]));
            circle( cv_ptr->image, center, 2, CV_RGB(0,255,255), -1, 8, 0);
            circle( cv_ptr->image, center, radius+20, CV_RGB(255,255,0), 3, 8, 0);
        }
        hello_cnt += 1;
    }

    vector<int> low_hsv = {low_h_, low_s_, low_v_};
    vector<int> high_hsv = {high_h_, high_s_, high_v_};

    inRange(cv_ptr->image, low_hsv, high_hsv, cv_ptr->image);
    blur(cv_ptr->image, cv_ptr->image, cv::Size(blur_k_size_, blur_k_size_));
    // if ((blur_k_size_ % 2) != 1)
    // {
    //     blur_k_size_++;
    // }
    // cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(blur_k_size_, blur_k_size_), 0);
    //cv::imshow("Before", cv_ptr->image);
    //cv::dilate(cv_ptr->image, cv_ptr->image, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * dil_k_size_ + 1, 2 * dil_k_size_ + 1), cv::Point(dil_k_size_, dil_k_size_)));
    // cv::erode(cv_ptr->image, cv_ptr->image, 5);

    //cv::namedWindow("circles", 1);
    imshow("Image Window", cv_ptr->image);
    waitKey(1);
}

int colorDetector::add(int a, int b)
{
    std::cout << a + b << std::endl;
    return a + b;
}

int main(int argc, char **argv) // main function
{
    ros::init(argc, argv, "ColorDetector"); // ros initiation. (argc, argv, "ColorDetector") = node name
    colorDetector cd;
    ros::spin(); // wait
    return 0;
}
