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
#include "segmentation.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdexcept>

ImageSegmenter::ImageSegmenter() : mouseData{ cv::Mat(), cv::Rect(), false } {}

void ImageSegmenter::onMouse(int event, int x, int y, int flags, void* userdata) {
    MouseData* data = static_cast<MouseData*>(userdata);
    
    if (event == cv::EVENT_LBUTTONDOWN) {
        data->isSelecting = true;
        data->roi.x = x;
        data->roi.y = y;
    }
    else if (event == cv::EVENT_MOUSEMOVE && data->isSelecting) {
        data->roi.width = x - data->roi.x;
        data->roi.height = y - data->roi.y;
        
        cv::Mat temp = data->image.clone();
        cv::rectangle(temp, data->roi, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Select ROI", temp);
    }
    else if (event == cv::EVENT_LBUTTONUP) {
        data->isSelecting = false;
        data->roi.width = x - data->roi.x;
        data->roi.height = y - data->roi.y;
    }
}

cv::Rect ImageSegmenter::selectROI(const cv::Mat& image) {
    mouseData.image = image.clone();
    mouseData.isSelecting = false;
    
    cv::namedWindow("Select ROI", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Select ROI", onMouse, &mouseData);
    
    cv::imshow("Select ROI", image);
    cv::waitKey(0);
    cv::destroyWindow("Select ROI");
    
    return mouseData.roi;
}

void ImageSegmenter::applyGrabCut(const cv::Mat& image, cv::Mat& output, const cv::Rect& roi) {
    if (image.empty()) {
        throw std::runtime_error("Error: Image is empty!");
    }

    cv::Rect finalRoi = roi;
    if (roi.empty()) {
        finalRoi = selectROI(image);
    }

    if (finalRoi.width <= 0 || finalRoi.height <= 0) {
        throw std::runtime_error("Error: Invalid ROI selected!");
    }

    cv::Mat mask, bgModel, fgModel;
    try {
        cv::grabCut(image, mask, finalRoi, bgModel, fgModel, 5, cv::GC_INIT_WITH_RECT);
        
        
        cv::Mat mask2 = (mask == cv::GC_FGD) | (mask == cv::GC_PR_FGD);
        mask2.convertTo(mask, CV_8U, 255);

        output = cv::Mat(image.size(), CV_8UC3, cv::Scalar(255, 255, 255));
        image.copyTo(output, mask);
    } catch (const cv::Exception& e) {
        throw std::runtime_error("GrabCut error: " + std::string(e.what()));
    }
}