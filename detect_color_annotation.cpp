#include <cv_bridge/cv_bridge.h>			// converts between ROS Image messages and OpenCV images
#include "detect_color.h"				// include header file
#include <image_transport/image_transport.h>		// transparent support for transporting images in low-bandwidth compressed formats
#include <iostream>					// Standard Input / Output Streams Library
#include <opencv2/highgui/highgui.hpp>			// include highgui.hpp
#include <ros/ros.h>					// include header file

colorDetector::colorDetector()
    : it_(nh_)
{
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &colorDetector::image_cb, this);
    low_h_ = 33;	// 색상(Hue) 최소값
    high_h_ = 81;	// 색상(Hue) 최대값
    low_s_ = 83;	// 채도(Saturation) 최소값
    high_s_ = 255;	// 채도(Saturation) 최대값
    low_v_ = 113;	// 명도(Value) 최소값
    high_v_ = 255;	// 명도(Value) 최대
    blur_k_size_ = 1;	
    dil_k_size_ = 1;
    cv::namedWindow("thresh_img", CV_WINDOW_AUTOSIZE); 			// 이미지를 출력하기 위한 콘솔 창 설정 cv::namedWindow(출력창 이름, 창 크기 옵션)
    cv::createTrackbar("low_h_", "thresh_img", &low_h_, 179);		// 트랙바 생성 cv::createTrackbar(트랙바명, 윈도우명, value, value 최댓값, onChange)
    cv::createTrackbar("high_h_", "thresh_img", &high_h_, 179);
    cv::createTrackbar("low_s_", "thresh_img", &low_s_, 255);
    cv::createTrackbar("high_s_", "thresh_img", &high_s_, 255);
    cv::createTrackbar("low_v_", "thresh_img", &low_v_, 255);
    cv::createTrackbar("high_v_", "thresh_img", &high_v_, 255);
    cv::createTrackbar("blur_k_size_", "thresh_img", &blur_k_size_, 15);
    cv::createTrackbar("dil_k_size_", "thresh_img", &dil_k_size_, 15);
}

void colorDetector::image_cb(const sensor_msgs::ImageConstPtr &msg) // 
{
    cv_bridge::CvImagePtr cv_ptr;
    try 									// 예외가 발생하게되면 catch로 이동
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 	// Convert a sensor_msgs::Image message to an OpenCV-compatible CvImage, copying the image data. 
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());				// 에러 메시지 출력 cv_bridge exception:
        return;
    }

    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));	// draws a simple or filled circle with a given center and radius.
    cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2HSV);			// Converts an image from one color space to another.

    std::vector<int> low_hsv = {low_h_, low_s_, low_v_};			// 동적 크기를 가진 배열을 캡슐화한 연속적인(sequence) 컨테이너. low_hsv 값 저
    std::vector<int> high_hsv = {high_h_, high_s_, high_v_};			// high_hsv 값 저장

    cv::inRange(cv_ptr->image, low_hsv, high_hsv, cv_ptr->image);		// 함수는 그 범위안에 들어가게되면 0으로 만들어주고 나머지는 1로 만들어 흑백사진을 만든다.
    cv::blur(cv_ptr->image, cv_ptr->image, cv::Size(blur_k_size_, blur_k_size_));	// Blurs an image using the normalized box filter.
    // if ((blur_k_size_ % 2) != 1)
    // {
    //     blur_k_size_++;
    // }
    // cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(blur_k_size_, blur_k_size_), 0);
    cv::imshow("Before", cv_ptr->image); 				// cv::imshow("윈도우 이름", 불러올 이미지가 저장된 Mat 형식)
    cv::dilate(cv_ptr->image, cv_ptr->image, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * dil_k_size_ + 1, 2 * dil_k_size_ + 1), cv::Point(dil_k_size_, dil_k_size_))); 							//팽창연산 (input array, output array, kernel)
	

     cv::erode(cv_ptr->image, cv_ptr->image, 5); 			// Erodes an image by using a specific structuring element.
    cv::imshow("Image Window", cv_ptr->image); 				// cv::imshow("윈도우 이름", 불러올 이미지가 저장된 Mat 형식)
    cv::waitKey(1); 							// 1ms 기다린 후 다음 코드 실행
}

int colorDetector::add(int a, int b) // 
{
    std::cout << a + b << std::endl;		// std::cout 화면에 그 값을 출력
    return a + b;				// std::endl 여러 줄을 출력
}

int main(int argc, char **argv) 		// main function
{
    ros::init(argc, argv, "ColorDetector"); 	// 노드명 초기화
    colorDetector cd;
    ros::spin(); 				// subscriber() 호출 후에는 ros::spin() 함수를 이용하여 반복 구독을 수행
    return 0;
}
