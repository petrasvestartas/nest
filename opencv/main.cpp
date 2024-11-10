#include <opencv2/opencv.hpp>
#include <iostream>

// https://docs.opencv.org/4.x/d9/df8/tutorial_root.html

int main(){
    std::string imagePath = "C:/brg/2_code/nest/opencv/image.jpg";
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
    cv::imshow("Display window", image);
    cv::waitKey(0);
    return 0;
}