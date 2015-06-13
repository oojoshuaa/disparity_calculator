#include "stereo.h"

namespace stereo
{

DisparityCalculator::DisparityCalculator()
{
  correspondence_points_file_ = "none";
  step_size_x_ = 70;
  step_size_y_ = 70;
  window_width_ = 50;
  window_height_ = 50;
  extra_padding_ = 5;
  
}

void DisparityCalculator::ComputeDisparity(std::string im_left_filename, std::string im_right_filename, int F_method,
        int rectification_method, int matching_method)
{
  LoadImages(im_left_filename, im_right_filename);
  GetF(F_method);
  Rectify(rectification_method);
  Match(matching_method);
}

void DisparityCalculator::GetRangeInsideImage(std::vector<double>& range, int which_image, double y_coord)
{
  std::vector<cv::Point2f> corners(4);

  if(which_image == kLeftImage)
  {
    corners[0].x = top_left_im_left_.at<double>(0, 0);
    corners[0].y = top_left_im_left_.at<double>(1, 0);
    corners[1].x = top_right_im_left_.at<double>(0, 0);
    corners[1].y = top_right_im_left_.at<double>(1, 0);
    corners[2].x = bottom_right_im_left_.at<double>(0, 0);
    corners[2].y = bottom_right_im_left_.at<double>(1, 0);
    corners[3].x = bottom_left_im_left_.at<double>(0, 0);
    corners[3].y = bottom_left_im_left_.at<double>(1, 0);
  } else
  {
    corners[0].x = top_left_im_right_.at<double>(0, 0);
    corners[0].y = top_left_im_right_.at<double>(1, 0);
    corners[1].x = top_right_im_right_.at<double>(0, 0);
    corners[1].y = top_right_im_right_.at<double>(1, 0);
    corners[2].x = bottom_right_im_right_.at<double>(0, 0);
    corners[2].y = bottom_right_im_right_.at<double>(1, 0);
    corners[3].x = bottom_left_im_right_.at<double>(0, 0);
    corners[3].y = bottom_left_im_right_.at<double>(1, 0);
  }

  double x_intercept_init[4] = {
    (y_coord - corners[0].y)*(corners[3].x - corners[0].x) / (corners[3].y - corners[0].y) + corners[0].x,
    (y_coord - corners[1].y)*(corners[2].x - corners[1].x) / (corners[2].y - corners[1].y) + corners[1].x,
    (y_coord - corners[0].y)*(corners[1].x - corners[0].x) / (corners[1].y - corners[0].y) + corners[0].x,
    (y_coord - corners[3].y)*(corners[2].x - corners[3].x) / (corners[2].y - corners[3].y) + corners[3].x
  };
  std::vector<double> x_intercept(x_intercept_init, x_intercept_init + 4);
  std::sort(x_intercept.begin(), x_intercept.end());

  for (int i = 0; i < 3; i++)
  {
    if(IsInConvexPoly(corners, (x_intercept[i] + x_intercept[i + 1]) / 2, y_coord ))
    {
      range.push_back(x_intercept[i]);
      range.push_back(x_intercept[i + 1]);
      break;
    }
  }

}

bool DisparityCalculator::IsInConvexPoly(std::vector<cv::Point2f> vertex, double x, double y)
{
  int prev_side = 0, side;

  for (int i = 0; i < vertex.size(); i++)
  {
    cv::Point2f a = vertex[i];
    cv::Point2f b = vertex[(i + 1) % vertex.size()];
    cv::Point2f P(x, y);

    double cross = (b - a).x * (P - a).y - (b - a).y * (P - a).x;
    if (cross < 0) side = 1;
    else if(cross > 0) side = 2;
    else side = 0;

    if (side == 0)return false;
    else if(prev_side == 0)prev_side = side;
    else if(prev_side != side) return false;

  }
  return true;

}

void DisparityCalculator::Match(int method)
{ 
  for (double i = window_height_ / 2; i < im_left_rectified_.rows - window_height_ / 2; i += step_size_y_)
  {
    std::vector<double> range_left_im;
    GetRangeInsideImage(range_left_im, kLeftImage, i);

    //TEMP
    cv::rectangle(im_left_rectified_, cv::Point2f(range_left_im[0], i - window_height_ / 2), cv::Point2f(range_left_im[1], i + window_height_ / 2), CV_RGB(255, 0, 0), 2);
    //
    std::vector<cv::Point2f> points_in_band;
    std::vector<cv::Point3f> epilines_for_points;
    for(double j=range_left_im[0]+window_width_/2; j<= range_left_im[1]-window_width_/2; j+=step_size_x_)
      points_in_band.push_back(cv::Point2f(j, i));
    cv::computeCorrespondEpilines(points_in_band, 1, F_after_transform_, epilines_for_points);
    
    for (double j = range_left_im[0] + window_width_ / 2, k=0; j <= range_left_im[1] - window_width_ / 2; j += step_size_x_,k++)
    {
      //TEMP
      cv::rectangle(im_left_rectified_, cv::Point2f(j - window_width_/2, i - window_height_ / 2), cv::Point2f(j+window_width_/2, i + window_height_ / 2), CV_RGB(255, 0, 255), 1);
      //
      double y_constraint = -epilines_for_points[k].z/epilines_for_points[k].y;
      cv::Rect template_region(j-window_width_/2, i-window_height_/2 ,window_width_, window_height_);
      
      std::vector<double> range_right_im;
      GetRangeInsideImage(range_right_im, kRightImage, y_constraint);
      cv::Rect looking_region(range_right_im[0], y_constraint-window_height_/2-extra_padding_, range_right_im[1]-range_right_im[0], window_height_+extra_padding_);
      
      //TODO: subMat with Rects and matching...
    }

  }
  //TEMP
  cv::namedWindow("Im", cv::WINDOW_AUTOSIZE);
  cv::imshow("Im", im_left_rectified_);
  cv::imwrite("im.jpg", im_left_rectified_);
  cv::waitKey(0);
  //
}

void DisparityCalculator::Rectify(int method)
{
  switch(method)
  {
    case kRectificationMethodHartley:
      cv::stereoRectifyUncalibrated(points_im_left_, points_im_right_, F_, im_left_.size(), H1_, H2_);
      break;
    case kRectificationMethodMallon:
      //TODO: Implement it.
      break;
  }
  top_left_im_right_ = H1_ * (cv::Mat_<double>(3, 1) << 0, 0, 1);
  top_right_im_right_ = H1_ * (cv::Mat_<double>(3, 1) << im_left_.cols, 0, 1);
  bottom_right_im_right_ = H1_ * (cv::Mat_<double>(3, 1) << im_left_.cols, im_left_.rows, 1);
  bottom_left_im_right_ = H1_ * (cv::Mat_<double>(3, 1) << 0, im_left_.rows, 1);
  top_left_im_right_ = (top_left_im_right_)*(1 / top_left_im_right_.at<double>(2, 0));
  top_right_im_right_ = (top_right_im_right_)*(1 / top_right_im_right_.at<double>(2, 0));
  bottom_right_im_right_ = (bottom_right_im_right_)*(1 / bottom_right_im_right_.at<double>(2, 0));
  bottom_left_im_right_ = (bottom_left_im_right_)*(1 / bottom_left_im_right_.at<double>(2, 0));
  double L = std::min(std::min(std::min(top_left_im_right_.at<double>(0, 0), top_right_im_right_.at<double>(0, 0)),
          bottom_right_im_right_.at<double>(0, 0)), bottom_left_im_right_.at<double>(0, 0));
  double R = std::max(std::max(std::max(top_left_im_right_.at<double>(0, 0), top_right_im_right_.at<double>(0, 0)),
          bottom_right_im_right_.at<double>(0, 0)), bottom_left_im_right_.at<double>(0, 0));
  double T = std::min(std::min(std::min(top_left_im_right_.at<double>(1, 0), top_right_im_right_.at<double>(1, 0)),
          bottom_right_im_right_.at<double>(1, 0)), bottom_left_im_right_.at<double>(1, 0));
  double B = std::max(std::max(std::max(top_left_im_right_.at<double>(1, 0), top_right_im_right_.at<double>(1, 0)),
          bottom_right_im_right_.at<double>(1, 0)), bottom_left_im_right_.at<double>(1, 0));
  translated_H1_ = (cv::Mat_<double>(3, 3) << 1, 0, -L, 0, 1, -T, 0, 0, 1) * H1_;
  cv::warpPerspective(im_left_, im_left_rectified_, translated_H1_, cv::Size_<double>(std::max(R - L, L - R), std::max(B - T, T - B) ));

  top_left_im_left_ = translated_H1_ * (cv::Mat_<double>(3, 1) << 0, 0, 1);
  top_right_im_left_ = translated_H1_ * (cv::Mat_<double>(3, 1) << im_left_.cols, 0, 1);
  bottom_right_im_left_ = translated_H1_ * (cv::Mat_<double>(3, 1) << im_left_.cols, im_left_.rows, 1);
  bottom_left_im_left_ = translated_H1_ * (cv::Mat_<double>(3, 1) << 0, im_left_.rows, 1);
  top_left_im_left_ = (top_left_im_left_)*(1 / top_left_im_left_.at<double>(2, 0));
  top_right_im_left_ = (top_right_im_left_)*(1 / top_right_im_left_.at<double>(2, 0));
  bottom_right_im_left_ = (bottom_right_im_left_)*(1 / bottom_right_im_left_.at<double>(2, 0));
  bottom_left_im_left_ = (bottom_left_im_left_)*(1 / bottom_left_im_left_.at<double>(2, 0));

  top_left_im_right_ = H2_ * (cv::Mat_<double>(3, 1) << 0, 0, 1);
  top_right_im_right_ = H2_ * (cv::Mat_<double>(3, 1) << im_right_.cols, 0, 1);
  bottom_right_im_right_ = H2_ * (cv::Mat_<double>(3, 1) << im_right_.cols, im_right_.rows, 1);
  bottom_left_im_right_ = H2_ * (cv::Mat_<double>(3, 1) << 0, im_right_.rows, 1);
  top_left_im_right_ = (top_left_im_right_)*(1 / top_left_im_right_.at<double>(2, 0));
  top_right_im_right_ = (top_right_im_right_)*(1 / top_right_im_right_.at<double>(2, 0));
  bottom_right_im_right_ = (bottom_right_im_right_)*(1 / bottom_right_im_right_.at<double>(2, 0));
  bottom_left_im_right_ = (bottom_left_im_right_)*(1 / bottom_left_im_right_.at<double>(2, 0));
  L = std::min(std::min(std::min(top_left_im_right_.at<double>(0, 0), top_right_im_right_.at<double>(0, 0)),
          bottom_right_im_right_.at<double>(0, 0)), bottom_left_im_right_.at<double>(0, 0));
  R = std::max(std::max(std::max(top_left_im_right_.at<double>(0, 0), top_right_im_right_.at<double>(0, 0)),
          bottom_right_im_right_.at<double>(0, 0)), bottom_left_im_right_.at<double>(0, 0));
  T = std::min(std::min(std::min(top_left_im_right_.at<double>(1, 0), top_right_im_right_.at<double>(1, 0)),
          bottom_right_im_right_.at<double>(1, 0)), bottom_left_im_right_.at<double>(1, 0));
  B = std::max(std::max(std::max(top_left_im_right_.at<double>(1, 0), top_right_im_right_.at<double>(1, 0)),
          bottom_right_im_right_.at<double>(1, 0)), bottom_left_im_right_.at<double>(1, 0));
  translated_H2_ = (cv::Mat_<double>(3, 3) << 1, 0, -L, 0, 1, -T, 0, 0, 1) * H2_;
  cv::warpPerspective(im_right_, im_right_rectified_, translated_H2_, cv::Size_<double>(std::max(R - L, L - R), std::max(B - T, T - B) ));

  top_left_im_right_ = translated_H2_ * (cv::Mat_<double>(3, 1) << 0, 0, 1);
  top_right_im_right_ = translated_H2_ * (cv::Mat_<double>(3, 1) << im_right_.cols, 0, 1);
  bottom_right_im_right_ = translated_H2_ * (cv::Mat_<double>(3, 1) << im_right_.cols, im_right_.rows, 1);
  bottom_left_im_right_ = translated_H2_ * (cv::Mat_<double>(3, 1) << 0, im_right_.rows, 1);
  top_left_im_right_ = (top_left_im_right_)*(1 / top_left_im_right_.at<double>(2, 0));
  top_right_im_right_ = (top_right_im_right_)*(1 / top_right_im_right_.at<double>(2, 0));
  bottom_right_im_right_ = (bottom_right_im_right_)*(1 / bottom_right_im_right_.at<double>(2, 0));
  bottom_left_im_right_ = (bottom_left_im_right_)*(1 / bottom_left_im_right_.at<double>(2, 0));
  
  F_after_transform_ = translated_H2_.inv().t()*F_*translated_H1_.inv();
}

void DisparityCalculator::LoadImages(std::string im_left_filename, std::string im_right_filename)
{
  im_left_ = cv::imread(im_left_filename);
  im_right_ = cv::imread(im_right_filename);
  if(! im_left_.data )
  {
    std::cout <<  "Could not open or find the image" << std::endl ;
    exit(1);
  }
  if(! im_right_.data )
  {
    std::cout <<  "Could not open or find the image" << std::endl ;
    exit(1);
  }
}

void DisparityCalculator::GetF(int method)
{
  if(correspondence_points_file_ == "none")
  {
    //TODO: get matching points like Mohamad does.
  } else
  {
    GetPointsFromFile(correspondence_points_file_, points_im_left_, points_im_right_);
  }
  switch(method)
  {
    case kFMatNPoint:
      if(points_im_left_.size() == 7) F_ = cv::findFundamentalMat(points_im_left_, points_im_right_, CV_FM_7POINT);
      else F_ = cv::findFundamentalMat(points_im_left_, points_im_right_, CV_FM_8POINT);
      break;
    case kFMatLMedS:
      F_ = cv::findFundamentalMat(points_im_left_, points_im_right_, CV_FM_LMEDS);
      break;
    case kFMatRANSAC:
      F_ = cv::findFundamentalMat(points_im_left_, points_im_right_, CV_FM_RANSAC);
      break;
    default:
      break;
  }
}

void DisparityCalculator::GetPointsFromFile(std::string filename, std::vector<cv::Point2f>& points_im1,
        std::vector<cv::Point2f>& points_im2)
{
  std::ifstream correspondence_file(filename.c_str());
  if(!correspondence_file.is_open())
  {
    std::cout << "Unable to open correspondence point coords file" << std::endl;
    exit(1);
  }
  std::string line;
  while(std::getline(correspondence_file, line))
  {
    if(line[0] == '%');
    else
    {
      cv::Point2f t1, t2;
      std::stringstream ss;
      ss << line;
      ss >> t1.x >> t1.y >> t2.x >> t2.y;
      points_im1.push_back(t1);
      points_im2.push_back(t2);
    }
  }
}

void DisparityCalculator::correspondence_points_file(std::string filename)
{
  correspondence_points_file_ = filename;
}

}