#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <vector>

class twodtree{
public:
  vector<cv::Point2f> pt;
  twodtree(vector<cv::Point2f> pt){
    pt=pt;
    
  };
  void build();
}