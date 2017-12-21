#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
#ifndef KDTREE
#define KDTREE
struct point{
  double x,y;
  
};
class twodtree{
public:
  vector<point> pts;
  twodtree(){
    
    
  };
  void build();
  void query();
  
  
};




#endif

