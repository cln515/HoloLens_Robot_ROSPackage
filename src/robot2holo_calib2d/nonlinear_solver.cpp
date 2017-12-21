#include "nonlinear_solver.h"

Eigen::Matrix3d axisRot2R(double rx, double ry, double rz){
  Eigen::Matrix4d R,rotx,roty,rotz;
  double sinv,cosv;
  sinv=sin(rx),cosv=cos(rx);



  rotx<<1,0,0,0,0,cosv,-sinv,0,0,sinv,cosv,0,0,0,0,1;
  sinv=sin(ry);cosv=cos(ry);
  roty<<cosv,0,sinv,0,0,1,0,0,-sinv,0,cosv,0,0,0,0,1;
  sinv=sin(rz);cosv=cos(rz);
  rotz<<cosv,-sinv,0,0,sinv,cosv,0,0,0,0,1,0,0,0,0,1;
  R=rotx*roty*rotz;
  Eigen::Matrix3d retMat=R.block(0,0,3,3);
  return retMat;
}

void R2axisRot(Eigen::Matrix3d R,double& rx,double& ry,double& rz){
  ry=asin(R(0,2));
  rx=-atan2(R(1,2),R(2,2));
  rz=-atan2(R(0,1),R(0,0));
}


int test_solve() {
  //google::InitGoogleLogging(argv[0]);

  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double x = 0.5;
  const double initial_x = x;

  // Build the problem.
  Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  problem.AddResidualBlock(cost_function, NULL, &x);

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x
            << " -> " << x << "\n";
  return 0;
}
