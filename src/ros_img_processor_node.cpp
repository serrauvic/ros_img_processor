#include "ros_img_processor_node.h"
#include "circle_detector.h"

//OpenCV
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

//constants
//const int GAUSSIAN_BLUR_SIZE        = 11;
//const double GAUSSIAN_BLUR_SIGMA    = 2;
//const double CANNY_EDGE_TH          = 150;
//const double HOUGH_ACCUM_RESOLUTION = 2;
//const double MIN_CIRCLE_DIST        = 30;
//const double HOUGH_ACCUM_TH         = 70;
//const int MIN_RADIUS                = 20;
//const int MAX_RADIUS                = 100;

RosImgProcessorNode::RosImgProcessorNode() :
    nh_(ros::this_node::getName()),
    img_tp_(nh_)
{
	//loop rate [hz], Could be set from a yaml file
	rate_=10;

	//sets publishers
	image_pub_ = img_tp_.advertise("image_out", 100);
  ray_direction_circle_pub = nh_.advertise<geometry_msgs::Vector3>("center_ray_direction", 1);

  ray_direction_ = (cv::Mat_<double>(3,1) << 0, 0, 0) ;

	//sets subscribers
	image_subs_ = img_tp_.subscribe("image_in", 1, &RosImgProcessorNode::imageCallback, this);
	camera_info_subs_ = nh_.subscribe("camera_info_in", 100, &RosImgProcessorNode::cameraInfoCallback, this);
}

RosImgProcessorNode::~RosImgProcessorNode()
{
    //
}

void RosImgProcessorNode::process()
{
    cv::Rect_<int> box;

    //check if new image is there
    if ( cv_img_ptr_in_ != nullptr )
    {
        //copy the input image to the out one
        cv_img_out_.image = cv_img_ptr_in_->image;

        //find the ball
    		//TODO bucar un cercle

    		//find the direction vector
    		//TODO amb el cerlce anterior agafem el centre i calculem la d (Ray Direction (in camera frame reference) mirar les diapos
        //https://campus.uvic.cat/aules/pluginfile.php/360761/mod_resource/content/1/PerceptionSystems_session3.pdf
        // basicament es aplicar la inversa de la matriu K pel punt del centre.
        // (per veure la matriu k) rostopic echo /usb_cam/camera_info
        //
        // vector3.msg.
        // com a punt a part podriem publicar aquesta de en un  missatge com un talker del tutorial 1 que vam fer.
        // publicar un marker per tal que des de rviz es pugui veure la fletxa del vector direcció.
        //
    std::vector<cv::Vec3f> circles;
    cv::Point center;
    //cv::Mat gray_image;
    int radius;


    //clear previous circles
    //circles.clear();
    // If input image is RGB, convert it to gray
    //cv::cvtColor(cv_img_out_.image, gray_image, CV_BGR2GRAY);
    //Reduce the noise so we avoid false circle detection
    //cv::GaussianBlur( gray_image, gray_image, cv::Size(GAUSSIAN_BLUR_SIZE, GAUSSIAN_BLUR_SIZE), GAUSSIAN_BLUR_SIGMA );
    //Apply the Hough Transform to find the circles
    //cv::HoughCircles( gray_image, circles, CV_HOUGH_GRADIENT, HOUGH_ACCUM_RESOLUTION, MIN_CIRCLE_DIST, CANNY_EDGE_TH, HOUGH_ACCUM_TH, MIN_RADIUS, MAX_RADIUS );


    Hough_Transform::calculate(cv_img_out_.image, circles);

    //draw circles on the image
    for(unsigned int ii = 0; ii < circles.size(); ii++ )
    {
        if ( circles[ii][0] != -1 )
        {
                Img_Circle::get_center_coordinates(circles[ii], center, radius);
                //center = cv::Point(cvRound(circles[ii][0]), cvRound(circles[ii][1]));
                //radius = cvRound(circles[ii][2]);
                //cv::circle(cv_img_out_.image, center, 5, cv::Scalar(128, 255, 0), -1, 8, 0 );// circle center in green
                //cv::circle(cv_img_out_.image, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0 );// circle perimeter in red

                draw_clircle(center, radius, true/*draw circle center coordinates*/);

                // circle center point x and y coordinates.
                //std::ostringstream stringStream;
                //stringStream  << "  x:" << center.x << "\n" << "y:" << center.y;
                // print circle center coordinates
                //cv::putText(cv_img_out_.image, stringStream.str(), center, cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0, 255, 0), 2, 0.5);

                Img_Circle::get_ray_direction(matrixK_, center, ray_direction_);

                //cv::Mat kinverted;
                //cv::invert(matrixK_, kinverted, cv::DECOMP_LU) ;

                //ray_direction = (cv::Mat_<double>(3,1) << 0, 0, 0) ;
                //cv::Mat centerPoint = (cv::Mat_<double>(3,1) << center.x, center.y, 1.0);
				        //ray_direction = kinverted * centerPoint;
        }
    }


        //sets and draw a bounding box around the ball
        box.x = (cv_img_ptr_in_->image.cols/2)-10;
        box.y = (cv_img_ptr_in_->image.rows/2)-10;
        box.width = 20;
        box.height = 20;
        cv::rectangle(cv_img_out_.image, box, cv::Scalar(0,255,255), 3);
    }

    //reset input image
    cv_img_ptr_in_ = nullptr;
}
void RosImgProcessorNode::draw_clircle(const cv::Point & center, const int radius, bool draw_center_coordinates)
{
  cv::circle(cv_img_out_.image, center, 5, cv::Scalar(128, 255, 0), -1, 8, 0 );// circle center in green
  cv::circle(cv_img_out_.image, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0 );// circle perimeter in red

  if (draw_center_coordinates)
  {
    // circle center point x and y coordinates.
    std::ostringstream stringStream;
    stringStream  << "  x:" << center.x << "\n" << "y:" << center.y;
    // print circle center coordinates
    cv::putText(cv_img_out_.image, stringStream.str(), center, cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0, 255, 0), 2, 0.5);
  }
}
void RosImgProcessorNode::publish()
{
    //image_raw topic
	if(cv_img_out_.image.data)
	{
	    cv_img_out_.header.seq ++;
	    cv_img_out_.header.stamp = ros::Time::now();
	    cv_img_out_.header.frame_id = "camera";
	    cv_img_out_.encoding = img_encoding_;
	    image_pub_.publish(cv_img_out_.toImageMsg());

      // publish center ray direction.
      geometry_msgs::Vector3 direction;
      direction.x = ray_direction_.at<double>(0, 0);
      direction.y = ray_direction_.at<double>(1, 0);
      direction.z = ray_direction_.at<double>(2, 0);
      ray_direction_circle_pub.publish(direction);

	}
}

double RosImgProcessorNode::getRate() const
{
    return rate_;
}

void RosImgProcessorNode::imageCallback(const sensor_msgs::ImageConstPtr& _msg)
{
    try
    {
        img_encoding_ = _msg->encoding;//get image encodings
        cv_img_ptr_in_ = cv_bridge::toCvCopy(_msg, _msg->encoding);//get image
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("RosImgProcessorNode::image_callback(): cv_bridge exception: %s", e.what());
        return;
    }
}

void RosImgProcessorNode::cameraInfoCallback(const sensor_msgs::CameraInfo & _msg)
{
	matrixP_ = (cv::Mat_<double>(3,3) << _msg.P[0],_msg.P[1],_msg.P[2],
                                        _msg.P[3],_msg.P[4],_msg.P[5],
                                        _msg.P[6],_msg.P[7],_msg.P[8]);
	//std::cout << matrixP_ << std::endl;

  matrixK_ = (cv::Mat_<double>(3,3) << _msg.K[0],_msg.K[1],_msg.K[2],
                                        _msg.K[3],_msg.K[4],_msg.K[5],
                                        _msg.K[6],_msg.K[7],_msg.K[8]);
//std::cout << matrixK_ << std::endl;
}
