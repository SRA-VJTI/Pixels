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
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include "interpolation.hpp"

cv::Scalar_<uint8_t> getPixel(cv::Mat imageData, int y, int x, int channels)
{
    /* Fetches the pixel corresponding to the coordinate x,y .
       OpenCV uses BGR Format and this leads to the following convention:

       Returns a cv Scalar_ object of type unisigned integer (0-256)

       Arguments :
       cv :: Mat : Matrix representing image
       int x : Row number of the pixel indexed from 0
       int y : Column number of the pixel indexed from 0
       int channels : No of channels in image

*/
    uint8_t *pixelPtr = (uint8_t *)imageData.data; // ptr to access data of image
    cv::Scalar_<uint8_t> bgrPixel;                 // cv::Scalar object to store pixel values of the 3 channels

    bgrPixel.val[0] = pixelPtr[y * imageData.cols * channels + x * channels + 0]; // Set channel 0
    bgrPixel.val[1] = pixelPtr[y * imageData.cols * channels + x * channels + 1]; // Set channel 1
    bgrPixel.val[2] = pixelPtr[y * imageData.cols * channels + x * channels + 2]; // Set channel 2

    return bgrPixel;
}
void setPixel(cv::Mat imageData, cv::Scalar_<uint8_t> v, int y, int x, int channels)
{
    /* Sets the pixel corresponding to the coordinate x,y a .
       OpenCV uses BGR Format and this leads to the following convention:


       Returns void

       Arguments :
       cv :: Mat imageData : Matrix representing image
       int v : Value of pixel to be set
       int x : Row number of the pixel indexed from 0
       int y : Column number of the pixel indexed from 0
       int channels : No of channels in image

*/

    uint8_t *pixelPtr = (uint8_t *)imageData.data; // ptr to access values in imageData array

    pixelPtr[y * imageData.cols * channels + x * channels + 0] = v.val[0]; // Set pixel at channel 0 and coordinates y,x
    pixelPtr[y * imageData.cols * channels + x * channels + 1] = v.val[1]; // Set pixel at channel 1 and coordinates y,x
    pixelPtr[y * imageData.cols * channels + x * channels + 2] = v.val[2]; // Set pixel at channel 2 and coordinates y,x
}

cv::Mat bilinear_interpolate(cv::Mat imageData, int width, int height, int channels)
{

    /* Interpolates the image to desired size using bilinear Interpolation


         Returns interpolated image as a cv::Mat object .

         Arguments :
         cv :: Mat imageData : Matrix representing image to be interpolated
         int width : width of resized image
         int height : height of resized image
         int channels : No of channels in image

  */

    int x_l, y_l, x_h, y_h;                                              // for storing coordinates of top left and bottom right neighbour
    float x_weight, y_weight;                                            // used in formula for interpolation
    cv::Mat newImage = cv::Mat::zeros(cv::Size(width, height), CV_8UC3); // image which will be returned
    double x_ratio = (double)(imageData.cols - 1) / (width - 1);         // used for mapping from new image to original image
    double y_ratio = (double)(imageData.rows - 1) / (height - 1);        // used for mapping from new image to original image

    cv::Scalar_<uint8_t> a, b, c, d, pixel; // Storing pixel values of neighbouring pixels and interpolated pixel

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            // Loop through every pixel and find coordinated of top left and bottom right pixel

            x_l = std::floor(x_ratio * j);
            y_l = std::floor(y_ratio * i);
            x_h = std::ceil(x_ratio * j);
            y_h = std::ceil(y_ratio * i);

            // weights to map pixel to original image
            x_weight = (x_ratio * j - x_l);
            y_weight = (y_ratio * i - y_l);

            // retrieve neighbouring pixels from original array
            a = getPixel(imageData, y_l, x_l, channels);
            b = getPixel(imageData, y_l, x_h, channels);
            c = getPixel(imageData, y_h, x_l, channels);
            d = getPixel(imageData, y_h, x_h, channels);
            for (int k = 0; k < channels; k++)
            {
                // loop through channels and set pixel value using bilinear interpolation formula
                pixel.val[k] = a.val[k] * (1 - x_weight) * (1 - y_weight) + b.val[k] * (x_weight) * (1 - y_weight) + c.val[k] * (1 - x_weight) * (y_weight) + d.val[k] * (x_weight) * (y_weight);
            }
            setPixel(newImage, pixel, i, j, channels); // Assign the new pixel formed in newImage
        }
    }
    return newImage; // return new Interpolated Image
}

cv::Mat nearest_neighbour_interpolate(cv::Mat imageData, int width, int height, int channels)
{
    /* Interpolates the image to desired size using nearest neighbour Interpolation


     Returns interpolated image as a cv::Mat object .

     Arguments :
     cv :: Mat imageData : Matrix representing image to be interpolated
     int width : width of resized image
     int height : height of resized image
     int channels : No of channels in image

*/
    cv ::Mat newImage = cv::Mat::zeros(cv::Size(width, height), CV_8UC3); // to store the new Interpolated image
    double x_ratio = (double)(imageData.cols - 1) / (width - 1);          // used for mapping from new image to original image
    double y_ratio = (double)(imageData.rows - 1) / (height - 1);         // used for mapping from new image to original image

    cv::Scalar_<uint8_t> nearest_pixel, pixel; // for storing nearest pixel in original image and pixel in new image

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            // nearest pixel is the pixel whose coordinates are integral and closest to mapped pixel
            nearest_pixel = getPixel(imageData, std::round(i * y_ratio), std::round(j * x_ratio), channels);

            // loop through channels and assign nearest pixel value to pixel
            for (int k = 0; k < channels; k++)
            {

                pixel.val[k] = nearest_pixel.val[k];
            }
            setPixel(newImage, pixel, i, j, channels); // Assign the new pixel formed in newImage
        }
    }

    return newImage; // return new Interpolated Image
}

