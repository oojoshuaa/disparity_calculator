/* 
 * File:   stereo.h
 * Author: joshua
 *
 * Created on June 10, 2015, 8:46 PM
 */

#ifndef STEREO_H
#define	STEREO_H

#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>
#include <algorithm>

namespace stereo
{
const int kFMatNPoint = 0;
const int kFMatRANSAC = 1;
const int kFMatLMedS=2;
const int kRectificationMethodHartley = 0;
const int kRectificationMethodMallon = 1;
const int kMatchMethodSqDiff = 0;
const int kLeftImage = 0;
const int kRightImage = 1;

class DisparityCalculator
{
public:
  DisparityCalculator();
  void correspondence_points_file(std::string filename);
  void ComputeDisparity(std::string im_left_filename, std::string im_right_filename, int F_method, int rectification_method,
        int matching_method);
private:
  std::vector<cv::Point2f> points_im_left_, points_im_right_;
  cv::Mat im_left_, im_right_;
  cv::Mat F_;
  cv::Mat F_after_transform_;
  cv::Mat H1_, H2_;
  cv::Mat translated_H1_, translated_H2_;
  cv::Mat im_left_rectified_, im_right_rectified_;
  cv::Mat top_left_im_right_, top_right_im_right_, bottom_left_im_right_, bottom_right_im_right_;
  cv::Mat top_left_im_left_, top_right_im_left_, bottom_left_im_left_, bottom_right_im_left_;
  std::string correspondence_points_file_;
  double step_size_x_, step_size_y_;
  double window_width_, window_height_;
  double extra_padding_;
  void LoadImages(std::string im_left_filename, std::string im_right_filename);
  void GetF(int method);
  void GetPointsFromFile(std::string filename, std::vector<cv::Point2f>& points_im1, std::vector<cv::Point2f>& points_im2);
  void Rectify(int method);
  void Match(int method);
  void GetRangeInsideImage(std::vector<double>& range, int which_image, double y_coord);
  bool IsInConvexPoly(std::vector<cv::Point2f> vertex, double x, double y);
};

}

#endif	/* STEREO_H */

