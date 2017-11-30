#ifndef CIRCLE_DETECTOR_H
#define CIRCLE_DETECTOR_H

//OpenCV
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"

namespace Hough_Transform
{
  //constants
  const int DEF_GAUSSIAN_BLUR_SIZE        = 11;
  const double DEF_GAUSSIAN_BLUR_SIGMA    = 2;
  const double DEF_CANNY_EDGE_TH          = 150;
  const double DEF_HOUGH_ACCUM_RESOLUTION = 2;
  const double DEF_MIN_CIRCLE_DIST        = 30;
  const double DEF_HOUGH_ACCUM_TH         = 70;
  const int DEF_MIN_RADIUS                = 20;
  const int DEF_MAX_RADIUS                = 100;

  void calculate(cv::Mat &image, std::vector<cv::Vec3f> & circles)
  {
    cv::Point center;
    cv::Mat gray_image;

    //clear previous circles
    circles.clear();
    // If input image is RGB, convert it to gray
    cv::cvtColor(image, gray_image, CV_BGR2GRAY);
    //Reduce the noise so we avoid false circle detection
    cv::GaussianBlur( gray_image, gray_image, cv::Size(DEF_GAUSSIAN_BLUR_SIZE, DEF_GAUSSIAN_BLUR_SIZE), DEF_GAUSSIAN_BLUR_SIGMA );
    //Apply the Hough Transform to find the circles
    cv::HoughCircles( gray_image, circles, CV_HOUGH_GRADIENT, DEF_HOUGH_ACCUM_RESOLUTION, DEF_MIN_CIRCLE_DIST, DEF_CANNY_EDGE_TH, DEF_HOUGH_ACCUM_TH, DEF_MIN_RADIUS, DEF_MAX_RADIUS );

  };

  void calculate(cv::Mat &image, std::vector<cv::Vec3f> & circles, int gaussian_blur_size, double gaussian_blur_sigma, double accum_resolution, double cirlce_dist, double canny, double accum_th, int min_rad, int max_rad)
  {
    cv::Point center;
    cv::Mat gray_image;

    //clear previous circles
    circles.clear();
    // If input image is RGB, convert it to gray
    cv::cvtColor(image, gray_image, CV_BGR2GRAY);
    //Reduce the noise so we avoid false circle detection
    cv::GaussianBlur( gray_image, gray_image, cv::Size(gaussian_blur_size, gaussian_blur_size), gaussian_blur_sigma );
    //Apply the Hough Transform to find the circles
    cv::HoughCircles( gray_image, circles, CV_HOUGH_GRADIENT, accum_resolution, cirlce_dist, canny, accum_th, min_rad, max_rad );

  };
} // Hough
namespace Hough_Circle
{
  // Reads center coordinates from a hough detected circle.
  void get_center_coordinates(const cv::Vec3f &circle, cv::Point &center, int &radius)
  {
    //Read center of the circle from Vec3f
    center = cv::Point(cvRound(circle[0]), cvRound(circle[1]));
    //Read radius of the center from Vec3f.
    radius = cvRound(circle[2]);
  };
  // calculate rady direction from Hough detected ciercle using the camera K (Intrinsic calibration matrix)
  // d = K^-1 * u.
  // where u is the center of the circle.
  void get_ray_direction(const cv::Mat &matrixK, const cv::Point &center, cv::Mat &ray_direction)
  {
    // Make room for the inverted K matrix.
    cv::Mat kinverted;
    // Inver the K matrix.
    cv::invert(matrixK, kinverted, cv::DECOMP_LU);
    // Make room and initialize ray direction value.
    ray_direction = (cv::Mat_<double>(3,1) << 0, 0, 0);
    // Convert CV Point type to Cv Mat type fixing z for calculate matrix product.
    cv::Mat centerPoint = (cv::Mat_<double>(3,1) << center.x, center.y, 1.0);
    // Calculate ray direction.
    ray_direction = kinverted * centerPoint;
  };
} // Hough_Circle

#endif
