
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
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdexcept>

int main()
{
    try
    {
        cv::Mat image = cv::imread("assets/image1.png");
        if (image.empty())
            throw std::runtime_error("Could not load image");

        ImageSegmenter segmenter;

        cv::Mat fg1, fg2, m1, m2;
        cv::Rect r1, r2;
        cv::Mat mask1, mask2;

        // Loop until valid segmentation for object 1
        while (true)
        {
            try
            {
                cv::Mat segmented1 = segmentObject(segmenter, image, "Select first ROI and press any key...");//segment object and get segmented image
                mask1 = smoothMask(createMask(segmented1)); //create mask and smooth it
                extractForeground(image, mask1, fg1, m1, r1);//extract foreground, mask, and bounding box
                break;
            }
            catch (const std::exception& e)
            {
                std::cerr << "First object segmentation failed: " << e.what() << ". Try again.\n";
            }
        }

        // Loop until valid segmentation for object 2
        while (true)
        {
            try
            {
                cv::Mat segmented2 = segmentObject(segmenter, image, "Select second ROI and press any key...");//segment object and get segmented image
                mask2 = smoothMask(createMask(segmented2));// create mask and smooth it
                extractForeground(image, mask2, fg2, m2, r2);//extract foreground, mask, and bounding box
                break;
            }
            catch (const std::exception& e)
            {
                std::cerr << "Second object segmentation failed: " << e.what() << ". Try again.\n";
            }
        }

        resizeObject(fg1, m1, r2.size());//resize foreground and mask of object 1 to match size of object 2
        resizeObject(fg2, m2, r1.size());//resize foreground and mask of object 2 to match size of object 1

        cv::Mat swapped = removeOriginalObjects(image, mask1, mask2, r1, r2);//remove original objects from the image and get the base for swapping

        pasteSwappedObjects(swapped, fg1, fg2, m1, m2, r1, r2);//paste the swapped objects onto the base image

        cv::imshow("Original", image);//  show the original image
        cv::imshow("Segmented SWAP", swapped);//show the swaped image
        cv::waitKey(0);
        cv::destroyAllWindows();

        std::cout << "Object swap completed successfully. Exiting..." << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}