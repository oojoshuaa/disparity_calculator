/* 
 * File:   main.cpp
 * Author: joshua
 *
 * Created on June 10, 2015, 8:38 PM
 */

#include <iostream>
#include "stereo.h"

using namespace std;

/* Test main file using DisparityCalculator class 
 */
int main(int argc, char** argv) 
{
  stereo::DisparityCalculator D;
  
  D.correspondence_points_file("./config/correspondence_points-1.txt");
  D.ComputeDisparity("./imgs/st1-1.jpg", "./imgs/st1-2.jpg", stereo::kFMatNPoint, stereo::kFMatNPoint, stereo::kMatchMethodSqDiff);
  
}

