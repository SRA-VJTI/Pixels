/*
MIT License

Copyright (c) 2023 Society of Robotics and Automation

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include "interpolation.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

int main(int argc, char **argv)
{
    cv::Mat image = cv::imread("./assets/pixel3.jpg");    // Reading image
    cv::Mat downscale_image = nearest_neighbour_interpolate(image, 100, 100, 3);    // Performing nearest neighbour interpolation on image
    cv::Mat upscale_image = bilinear_interpolate(image, 1000, 1000, 3);      // Performing bilinear interpolation on image
    cv::imwrite("./assets/pixeli3s.jpg", upscale_image);                    // Saving image
    cv::imwrite("./assets/pixeli3d.jpg", downscale_image);                    // Saving image

    cv::imshow("Original Image", image);                     // Display original image 
    cv::imshow("Interpolation UpScaling!", upscale_image);   // Display upscaled image
    cv::imshow("Interpolation DownScaling!", downscale_image); // Display  downscaled image
    
    cv::waitKey(0); 
    cv::destroyAllWindows();
}