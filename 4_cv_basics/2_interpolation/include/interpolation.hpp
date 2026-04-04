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
#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

cv::Scalar_<uint8_t> getPixel(cv::Mat img, int y, int x, int channels);                  // declaration of getPixel used to get pixel value
void setPixel(cv::Mat img, cv::Scalar_<uint8_t> v, int y, int x, int channels);          // declaration of setPixel used to set pixel value
cv::Mat bilinear_interpolate(cv::Mat img, int width, int height, int channels);          // declaration of bilinear_interpolate to resize image using bilinear interpolation
cv::Mat nearest_neighbour_interpolate(cv::Mat img, int width, int height, int channels); // declaration of nearest_neighbour_interpolate to resize image using nearest neighbour
