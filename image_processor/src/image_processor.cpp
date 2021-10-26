#include "image_processor/image_processor.h"
#include "image_processor/circle_detector.h"
#include "image_processor/camera.h"

//OpenCV
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

ImageProcessor::ImageProcessor()
{
  ray_direction_ = (cv::Mat_<double>(3,1) << 0, 0, 0) ;
}

ImageProcessor::~ImageProcessor()
{
    //
}

void ImageProcessor::setInputImage(cv_bridge::CvImagePtr img_ptr){
  cv_img_ptr_in_ = img_ptr;
}

void ImageProcessor::setCameraInfo(cv::Mat matrixP, cv::Mat matrixK){
  matrixP_ = matrixP;
  matrixK_ = matrixK;
}

void ImageProcessor::process()
{
    cv::Rect_<int> box;

    //check if new image is there
    if ( cv_img_ptr_in_ != nullptr )
    {
        //copy the input image to the out one
        cv_img_out_.image = cv_img_ptr_in_->image;
        // detected circles-
        std::vector<cv::Vec3f> circles;

        // detect circles in the image.
        Hough_Transform::calculate(cv_img_out_.image, circles);

        //draw circles on the image
        for(unsigned int ii = 0; ii < circles.size(); ii++ )
        {
            // if valid circle.
            if ( circles[ii][0] != -1 )
            {
                // center of the circle.
                cv::Point center;
                // radius of the circle.
                int radius;

                // get cirlce coodinates, center point and radius.
                Hough_Circle::get_center_coordinates(circles[ii], center, radius);
                // draw circle.
                draw_clircle(center, radius, true/*draw circle center coordinates*/);
                // calculate center circle ray direction from camera frame persepctive,
                // put the circle center point in the real world.
                Camera::get_ray_direction(matrixK_, center, ray_direction_);
                // draw vector.
                //draw_ray_direction_vector(center);
            }
        }

        //sets and draw a bounding box around the ball
        //box.x = (cv_img_ptr_in_->image.cols/2)-10;
        //box.y = (cv_img_ptr_in_->image.rows/2)-10;
        //box.width = 20;
        //box.height = 20;
        //cv::rectangle(cv_img_out_.image, box, cv::Scalar(0,255,255), 3);
    }

    //reset input image
    cv_img_ptr_in_ = nullptr;
}
void ImageProcessor::draw_clircle(const cv::Point & center, int radius, bool draw_center_coordinates)
{
  // circle center in yellow
  cv::circle(cv_img_out_.image, center, 5, cv::Scalar(255, 255, 0), -1, 8, 0 );
  // circle perimeter in purple.
  cv::circle(cv_img_out_.image, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_8, 0 );

  if (draw_center_coordinates)
  {
    // circle center point x and y coordinates.
    std::ostringstream stringStream;
    stringStream  << "  x:" << center.x << "\n" << " y:" << center.y;
    // print circle center coordinates
    cv::putText(cv_img_out_.image, stringStream.str(), center, cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 153, 51), 2, 0.5);
  }
}
void ImageProcessor::draw_ray_direction_vector(const cv::Point & center)
{
  // line from center circle
  cv::line( cv_img_out_.image, center, cv::Point( ray_direction_.at<double>(0, 0), ray_direction_.at<double>(1, 0) ), cv::Scalar( 110, 220, 0 ),  2, 8 );
}

cv_bridge::CvImage  ImageProcessor::getOutputImage(){
  return cv_img_out_;
}

cv::Mat ImageProcessor::getRayDirection(){

  return ray_direction_;
}
