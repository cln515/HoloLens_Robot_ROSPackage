/*
Copyright (c) 2017 Ryoichi Ishikawa. All rights reserved.

This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
*/


#include "linearSolver.h"

void linearCompute(std::vector<cv::Point2f> a,std::vector<cv::Point2f> b,Eigen::Matrix2d& R,Eigen::Vector2d& T){
//get centroid  
  cv::Point2f ga(0,0),gb(0,0);
  for(int i=0;i<a.size();i++){
      ga+=a.at(i);
      gb+=b.at(i);
  }
  ga=ga*(1.0/a.size());  gb=gb*(1.0/a.size());
  //
  Eigen::Vector2d vga,vgb;
  vga<<ga.x,ga.y;
  vgb<<gb.x,gb.y;
  Eigen::MatrixXd H(2,2);
  H.setZero();
  for(int i=0;i<a.size();i++){
    Eigen::Vector2d pa,pb;
    pa<<a.at(i).x,a.at(i).y;
    pb<<b.at(i).x,b.at(i).y;    
    Eigen::MatrixXd h_=(pa-vga)*(pb-vgb).transpose();
    H=H+h_;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix2d Hm;
  Eigen::Matrix2d uvt=svd.matrixU()*svd.matrixV().transpose();
  Hm<<1,0,0,uvt(0,0)*uvt(1,1)-uvt(0,1)*uvt(1,0);
  R=svd.matrixV()*Hm*svd.matrixU().transpose();
  
/*  if(R.determinant()<0){
    R.col(2)=-1*R.col(2);    
  }*/
  T=-R*vga+vgb;
  return;
};
