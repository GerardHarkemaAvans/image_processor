
//ros dependencies
#include "image_processor/image_processor_node.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

//node main
int main(int argc, char **argv)
{

      //init ros
      ros::init(argc, argv, "image_processor");

      //create ros wrapper object
      ImageProcessorRos imgp_ros;

      //set node loop rate
      ros::Rate loopRate(imgp_ros.getRate());

      //node loop
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce();

            //do things
           imgp_ros.process();

            //publish things
            imgp_ros.publish();

            //relax to fit output rate
            loopRate.sleep();
      }
      //exit program
      return 0;
}



ImageProcessorRos::ImageProcessorRos() :
    nh_(ros::this_node::getName()),
    img_tp_(nh_)
{
	//loop rate [hz], Could be set from a yaml file
	rate_=10;

	//sets publishers
	image_pub_ = img_tp_.advertise("image_out", 100);
  ray_direction_circle_pub   = nh_.advertise<geometry_msgs::Vector3>("center_ray_direction", 1);

  ray_direction_ = (cv::Mat_<double>(3,1) << 0, 0, 0);

	//sets subscribers
	image_subs_ = img_tp_.subscribe("image_in", 1, &ImageProcessorRos::imageCallback, this);
	camera_info_subs_ = nh_.subscribe("camera_info_in", 100, &ImageProcessorRos::cameraInfoCallback, this);
}


ImageProcessorRos::~ImageProcessorRos()
{

}

void ImageProcessorRos::process(){
  imgp.process();

}

void ImageProcessorRos::publish()
{
  cv_img_out_.image = imgp.getOutputImage();
  ray_direction_ = imgp.getRayDirection();

  //image_raw topic
	if(cv_img_out_.image.data)
	{
      //ROS_INFO("Publish image out");
	    cv_img_out_.header.seq ++;
	    cv_img_out_.header.stamp = ros::Time::now();
	    cv_img_out_.header.frame_id = "camera";
	    cv_img_out_.encoding = img_encoding_;
	    image_pub_.publish(cv_img_out_.toImageMsg());
      // publish center ray direction.

      geometry_msgs::Vector3 direction;

      direction.x = ray_direction_.at<double>(0, 0); //Minus added for matching the robot mounting bracket
      direction.y = -ray_direction_.at<double>(1, 0);
      direction.z = ray_direction_.at<double>(2, 0);

      ray_direction_circle_pub.publish(direction);
	}
}

double ImageProcessorRos::getRate() const
{
    return rate_;
}

void ImageProcessorRos::imageCallback(const sensor_msgs::ImageConstPtr& _msg)
{
  try
  {
      img_encoding_ = _msg->encoding;//get image encodings
      cv_img_ptr_in_ = cv_bridge::toCvCopy(_msg, _msg->encoding);//get image
      imgp.setInputImage(&cv_img_ptr_in_->image);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("ImageProcessorRos::image_callback(): cv_bridge exception: %s", e.what());
      return;
  }
}

void ImageProcessorRos::cameraInfoCallback(const sensor_msgs::CameraInfo & _msg)
{

  //Camera matrix
  cv::Mat matrixP_;
  cv::Mat matrixK_;

	matrixP_ = (cv::Mat_<double>(3,3) << _msg.P[0],_msg.P[1],_msg.P[2],
                                        _msg.P[3],_msg.P[4],_msg.P[5],
                                        _msg.P[6],_msg.P[7],_msg.P[8]);
	//std::cout << matrixP_ << std::endl;

  matrixK_ = (cv::Mat_<double>(3,3) << _msg.K[0],_msg.K[1],_msg.K[2],
                                        _msg.K[3],_msg.K[4],_msg.K[5],
                                        _msg.K[6],_msg.K[7],_msg.K[8]);

  imgp.setCameraInfo(matrixP_, matrixK_);
//std::cout << matrixK_ << std::endl;
}
