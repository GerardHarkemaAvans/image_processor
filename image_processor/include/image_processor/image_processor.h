#ifndef ROS_IMG_PROCESSOR_H
#define ROS_IMG_PROCESSOR_H

#include <opencv2/opencv.hpp>

//std C++
#include <iostream>

// do not use ROS functions/headers/etc. in this file

/** \brief Simple Image Processor
 *
 * Simple Image Processor with opencv calls
 *
 */

class ImageProcessor
{
    protected:

        //pointer to received (in) and published (out) images
        cv::Mat *cv_mat_ptr_in_ = nullptr;
        cv::Mat cv_mat_out_;

        // ray direction in camera frame.
        cv::Point circle_center_;

    		//Camera matrix
    		cv::Mat matrixP_;
        cv::Mat matrixK_;

        //image encoding label
        std::string img_encoding_;

        //wished process rate, [hz]
        double rate_;

    protected:
        // callbacks

        void draw_circle(const cv::Point & center, const int radius, bool draw_center_coordinates);

    public:
        /** \brief Constructor
        *
        * Constructor
        *
        */
        ImageProcessor();

        /** \brief Destructor
        *
        * Destructor
        *
        */
        ~ImageProcessor();

        void setInputImage(cv::Mat  *cv_mat_ptr);
        void setCameraInfo(cv::Mat matrixP, cv::Mat matrixK);

        /** \brief Process input image
        *
        * Process input image
        *
        **/
        void process();

        cv::Mat getOutputImage();
        cv::Point getCircleCenter();

};
#endif
