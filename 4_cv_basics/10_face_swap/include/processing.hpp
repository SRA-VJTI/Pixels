/* 
MIT License

Copyright (c) 2026 Society of Robotics and Automation

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

#ifndef PROCESSING_HPP
#define PROCESSING_HPP

#include "segmentation.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>

cv::Mat segmentObject(ImageSegmenter& segmenter, const cv::Mat& image, const std::string& message);

cv::Mat createMask(const cv::Mat& segmented);

int kernel_sum(const cv::Mat& source_image, int row, int col, int kernel_size);

cv::Mat erosion(cv::Mat source_image, cv::Mat output_image, int kernel_size);

cv::Mat dilation(cv::Mat source_image, cv::Mat output_image, int kernel_size);

cv::Mat smoothMask(const cv::Mat& inputMask);

void extractForeground(const cv::Mat& image, const cv::Mat& mask, cv::Mat& foreground, cv::Mat& croppedMask, cv::Rect& boundingBox);

void resizeObject(cv::Mat& object, cv::Mat& mask, const cv::Size& targetSize);

cv::Mat removeOriginalObjects(const cv::Mat& image, const cv::Mat& mask1, const cv::Mat& mask2, const cv::Rect& r1, const cv::Rect& r2);

void pasteSwappedObjects(cv::Mat& baseImage, const cv::Mat& fg1, const cv::Mat& fg2, const cv::Mat& m1, const cv::Mat& m2, const cv::Rect& r1, const cv::Rect& r2);

#endif