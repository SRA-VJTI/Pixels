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
#include "processing.hpp"
#include <iostream>
#include <stdexcept>

cv::Mat segmentObject(ImageSegmenter& segmenter, const cv::Mat& image, const std::string& message)
{
    std::cout << message << std::endl;
    cv::Mat segmented;
    segmenter.applyGrabCut(image, segmented);
    return segmented;
}

cv::Mat createMask(const cv::Mat& segmented)
{
    int height = segmented.rows;
    int width  = segmented.cols;

    cv::Mat mask = cv::Mat::zeros(segmented.size(), CV_8UC1);

    cv::Vec3b grey  = cv::Vec3b(230, 230, 230);
    cv::Vec3b white = cv::Vec3b(255, 255, 255);

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            cv::Vec3b pixel = segmented.at<cv::Vec3b>(i, j);
            if (pixel != grey && pixel != white)
                mask.at<uchar>(i, j) = 255;
        }
    }
    return mask;
}

int kernel_sum(const cv::Mat& source_image, int row, int col, int kernel_size)
{
    int sum  = 0;
    int half = kernel_size / 2;

    for (int i = row - half; i <= row + half; i++)
    {
        for (int j = col - half; j <= col + half; j++)
        {
            if (i >= 0 && i < source_image.rows && j >= 0 && j < source_image.cols)
                sum += source_image.at<uchar>(i, j);
        }
    }
    return sum;
}

cv::Mat erosion(cv::Mat source_image, cv::Mat output_image, int kernel_size)
{
    for (int i = 0; i < source_image.rows; i++)
    {
        for (int j = 0; j < source_image.cols; j++)
        {
            if (kernel_sum(source_image, i, j, kernel_size) != 255 * (kernel_size * kernel_size))
                output_image.at<uchar>(i, j) = cv::saturate_cast<uchar>(0);
            else
                output_image.at<uchar>(i, j) = cv::saturate_cast<uchar>(255);
        }
    }
    return output_image;
}

cv::Mat dilation(cv::Mat source_image, cv::Mat output_image, int kernel_size)
{
    for (int i = 0; i < source_image.rows; i++)
    {
        for (int j = 0; j < source_image.cols; j++)
        {
            if (kernel_sum(source_image, i, j, kernel_size) > 0)
                output_image.at<uchar>(i, j) = cv::saturate_cast<uchar>(255);
            else
                output_image.at<uchar>(i, j) = cv::saturate_cast<uchar>(0);
        }
    }
    return output_image;
}

cv::Mat smoothMask(const cv::Mat& inputMask)
{
    int kernel_size = 5;

    cv::Mat dilated = cv::Mat::zeros(inputMask.size(), inputMask.type());
    cv::Mat result  = cv::Mat::zeros(inputMask.size(), inputMask.type());

    dilated = dilation(inputMask, dilated, kernel_size);
    result  = erosion(dilated, result, kernel_size);

    return result;
}

void extractForeground(const cv::Mat& image, const cv::Mat& mask, cv::Mat& foreground, cv::Mat& croppedMask, cv::Rect& boundingBox)
{
    boundingBox = cv::boundingRect(mask);

    if (boundingBox.width == 0 || boundingBox.height == 0)
        throw std::runtime_error("Bounding box is empty");

    cv::Mat temp(image.size(), image.type(), cv::Scalar::all(0));
    image.copyTo(temp, mask);

    foreground  = temp(boundingBox).clone();
    croppedMask = mask(boundingBox).clone();
}

void resizeObject(cv::Mat& object, cv::Mat& mask, const cv::Size& targetSize)
{
    if (targetSize.width == 0 || targetSize.height == 0)
        throw std::runtime_error("Target size is invalid");

    cv::resize(object, object, targetSize, 0, 0, cv::INTER_LINEAR);
    cv::resize(mask,   mask,   targetSize, 0, 0, cv::INTER_NEAREST);
}

cv::Mat removeOriginalObjects(const cv::Mat& image, const cv::Mat& mask1, const cv::Mat& mask2, const cv::Rect& r1, const cv::Rect& r2)
{
    cv::Mat result = image.clone();

    result(r1).setTo(cv::Scalar(105, 172, 224), mask1(r1));
    result(r2).setTo(cv::Scalar(105, 172, 224), mask2(r2));

    return result;
}

void pasteSwappedObjects(cv::Mat& baseImage, const cv::Mat& fg1, const cv::Mat& fg2, const cv::Mat& m1, const cv::Mat& m2, const cv::Rect& r1, const cv::Rect& r2)
{
    cv::Mat layer1 = cv::Mat::zeros(baseImage.size(), baseImage.type());
    cv::Mat layer2 = cv::Mat::zeros(baseImage.size(), baseImage.type());

    fg1.copyTo(layer1(r2), m1);
    fg2.copyTo(layer2(r1), m2);

    cv::Mat maskL1, maskL2;
    cv::cvtColor(layer1, maskL1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(layer2, maskL2, cv::COLOR_BGR2GRAY);

    layer1.copyTo(baseImage, maskL1);
    layer2.copyTo(baseImage, maskL2);
}