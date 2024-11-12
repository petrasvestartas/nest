#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>

// https://docs.opencv.org/4.x/d9/df8/tutorial_root.html
// cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --config Release && .\build\Release\OpenCVExample.exe
int main(){
    // Creating a Mat object explicitly
    // Data type: CV_[The number of bits per item][Signed or Unsigned][Type Prefix]C[The channel number]
    // 8 - bit
    // U - unsigned
    // C - char
    // 3 - 3 channels
    cv::Mat M(2,2, CV_8UC3, cv::Scalar(0,0,255));
    std::cout  << M << std::endl << std::endl;

    // C++ Array in 3 dimensions - cannot be printed
    int sz[3] = {2,2,2};
    cv::Mat L(3, sz, CV_8UC(1), cv::Scalar::all(0));
    
    // cv::Mat::create
    M.create(4,4, CV_8UC(2));
    std::cout << M << std::endl << std::endl;

    // Create matrix with 1
    cv::Mat E = cv::Mat::ones(2, 2, CV_8UC1);
    std::cout << E << std::endl << std::endl;

    // Create matrix with 0
    cv::Mat Z = cv::Mat::zeros(2, 2, CV_8UC1);
    std::cout << Z << std::endl << std::endl;

    // Copy constructor, assignment operator does not copy the data
    cv::Mat Z2 = Z;
    Z2.at<uchar>(0,0) = 1;
    cv::Mat Z3(Z);
    Z3.at<uchar>(0,0) = 5;

    std::cout << Z2 << std::endl << std::endl;
    std::cout << Z3 << std::endl << std::endl;

    // Copy Matrix
    cv::Mat Z_copy = Z.clone();
    std::cout << Z_copy << std::endl << std::endl;
    cv::Mat Z_copy2;
    Z.copyTo(Z_copy2);

    // Create a large matrix for a black and white image and set some pixels to white and measure time and print the matrix and show the image
    double t = (double)cv::getTickCount();
    int rows = 10; // Number of rows
    int cols = 10; // Number of columns
    cv::Mat bwImage(rows, cols, CV_8UC1, cv::Scalar(0)); // Initialize with black (0)

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (i > 5) { // Example pattern: checkerboard
                bwImage.at<uchar>(i, j) = 255; // Set to white
            }
        }
    }

    std::cout << "Times passed in seconds: " << ((double)cv::getTickCount() - t)/cv::getTickFrequency()*1000 << std::endl;
    std::cout << "bwImage = " << std::endl << " " << bwImage << std::endl << std::endl;
    cv::imshow("Black and White Image", bwImage);
    cv::waitKey(0);
    return 0;
}