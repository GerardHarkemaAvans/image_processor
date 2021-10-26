#ifndef ROS_IMG_PROCESSOR_H
#define ROS_IMG_PROCESSOR_H

//std C++
#include <iostream>

//ROS headers for image I/O
//#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//#include "geometry_msgs/Vector3.h"

/** \brief Simple Image Processor
 *
 * Simple Image Processor with opencv calls
 *
 */
class ImageProcessor
{
    protected:

        //pointer to received (in) and published (out) images
        cv_bridge::CvImagePtr cv_img_ptr_in_;
        cv_bridge::CvImage cv_img_out_;
        // ray direction in camera frame.
        cv::Mat ray_direction_;

    		//Camera matrix
    		cv::Mat matrixP_;
        cv::Mat matrixK_;

        //image encoding label
        std::string img_encoding_;

        //wished process rate, [hz]
        double rate_;

    protected:
        // callbacks

        void draw_clircle(const cv::Point & center, const int radius, bool draw_center_coordinates);
        void draw_ray_direction_vector(const cv::Point & center);

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

        void setInputImage(cv_bridge::CvImagePtr img_ptr);
        void setCameraInfo(cv::Mat matrixP, cv::Mat matrixK);

        /** \brief Process input image
        *
        * Process input image
        *
        **/
        void process();

        cv_bridge::CvImage getOutputImage();
        cv::Mat getRayDirection();

};
#endif
